#!/usr/bin/python3

import dbus
import gatt
import struct
import threading
import time
import eventfd

NANO_SERVICE_UUID = '590d65c7-3a0a-4023-a05a-6aaf2f22441c'

NANO_MAG_ID  = '00000001-0000-1000-8000-00805f9b34fb'
NANO_GYRO_ID = '00000002-0000-1000-8000-00805f9b34fb'
NANO_ACC_ID  = '00000003-0000-1000-8000-00805f9b34fb'

class ManagerThread(threading.Thread):
    def __init__(self, manager):
        super().__init__()
        self.manager = manager

    def run(self):
        # Jumps to manager event loop
        self.manager.run()

class Nano33DeviceManager(gatt.DeviceManager):
    def __init__(self, adapter_name='hci0', discovery_callback=None):
        self.thread = ManagerThread(self)
        self.discovery_callback = discovery_callback
        super(Nano33DeviceManager, self).__init__(adapter_name=adapter_name)

    def device_discovered(self, device):
        if self.discovery_callback:
            self.discovery_callback(self, device)

    def start(self):
        self.thread.start()

    def stop(self):
        super().stop()

    def join(self):
        self.thread.join()

class Nano33Device(gatt.Device):
    mag  = [0.0, 0.0, 0.0, 0]
    acc  = [0.0, 0.0, 0.0, 0]
    gyro = [0.0, 0.0, 0.0, 0]

    def __init__(self, mac_address, manager, callback=None):
        self.lock = threading.Lock()
        self.eventfd = eventfd.EventFD()
        self.callback = callback
        super(Nano33Device, self).__init__(mac_address=mac_address, manager=manager)

    def services_resolved(self):
        super().services_resolved()

        device_information_service = next(
            s for s in self.services
            if s.uuid == NANO_SERVICE_UUID)

        for ch in device_information_service.characteristics:
            if ch.uuid == NANO_ACC_ID:
                ch.enable_notifications()
            elif ch.uuid == NANO_GYRO_ID:
                ch.enable_notifications()
            elif ch.uuid == NANO_MAG_ID:
                ch.enable_notifications()

    def characteristic_value_updated(self, characteristic, value):
        fmt = '<fffi'
        x, y, z, ts = struct.unpack(fmt, value)

        self.lock.acquire()
        if characteristic.uuid == NANO_ACC_ID:
            self.acc = [x, y, z, ts]
        elif characteristic.uuid == NANO_GYRO_ID:
            self.gyro = [x, y, z, ts]
        elif characteristic.uuid == NANO_MAG_ID:
            self.mag = [x, y, z, ts]
        self.eventfd.set()
        self.lock.release()
        if self.callback:
            self.callback(self)

    def get_data(self):
        self.lock.acquire()
        self.eventfd.clear()
        mag = self.mag
        gyro = self.gyro
        acc = self.acc
        self.lock.release()
        return acc, gyro, mag

import select
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--discover", action="store_true", help="Discover active DWM bluetooth devices")
    parser.add_argument("--adapter", type=str, default= "hci0", help="Bluetooth adapter to use")
    parser.add_argument("--readlocation", action="store_true", help="Read location from the dwm device")
    parser.add_argument("--continous", action="store_true", help="Continously read location from the device")
    parser.add_argument("--mac", type=str, help="Target device mac address")

    args = parser.parse_args()

    manager = Nano33DeviceManager(adapter_name=args.adapter)
    device = Nano33Device(mac_address=args.mac, manager=manager)

    device.connect()
    manager.start()

    ts = time.time()

    while True:
        r, w, e = select.select([device.eventfd], [], [])
        acc, gyro, mag = device.get_data()

        now = time.time()
        rate = 1 / (now - ts)
        ts = now

        print("acc = {x=%f y=%f z=%f ts=%d}, gyro = {x=%f y=%f z=%f ts=%d}, mag = {x=%f y=%f z=%f ts=%d}, rate %f" % \
              (acc[0], acc[1], acc[2], acc[3],
               gyro[0], gyro[1], gyro[2], gyro[3],
               mag[0], mag[1], mag[2], mag[3],
               rate))
