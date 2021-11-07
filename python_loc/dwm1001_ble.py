#!/usr/bin/env python3

import dbus
import gatt
import struct
import threading
import time
import eventfd

DWM_SERVICE_UUID = '680c21d9-c946-4c1f-9c11-baa1c21329e7'
DWM_LOCATION_CHARACTERISTIC_ID = '003bbdf2-c634-4b3d-ab56-7ec889b89a37'

class ManagerThread(threading.Thread):
    def __init__(self, manager):
        super().__init__()
        self.manager = manager

    def run(self):
        # Jumps to manager event loop
        self.manager.run()

class DwmDeviceManager(gatt.DeviceManager):
    def __init__(self, adapter_name='hci0', discovery_callback=None):
        self.thread = ManagerThread(self)
        self.discovery_callback = discovery_callback
        super(DwmDeviceManager, self).__init__(adapter_name=adapter_name)

    def device_discovered(self, device):
        if self.discovery_callback:
            self.discovery_callback(self, device)

    def start(self):
        self.thread.start()

    def stop(self):
        super().stop()

    def join(self):
        self.thread.join()

class DwmDevice(gatt.Device):
    def __init__(self, mac_address, manager, location_callback=None):
        self.lock = threading.Lock()
        self.eventfd = eventfd.EventFD()
        self.subscribe = True
        self.location_callback = location_callback
        super(DwmDevice, self).__init__(mac_address=mac_address, manager=manager)

    def services_resolved(self):
        super().services_resolved()
        self._read_location()

    def characteristic_value_updated(self, characteristic, value):
        location = self._decode_location(value)
        if not location:
            # Error
            return

        self.lock.acquire()
        self.location = location
        self.eventfd.set()
        self.lock.release()
        if self.location_callback:
            self.location_callback(self, location)

    def get_location(self):
        self.lock.acquire()
        self.eventfd.clear()
        location = self.location
        self.location = {}
        self.lock.release()
        return location

    def _read_location(self):
        device_information_service = next(
            s for s in self.services
            if s.uuid == DWM_SERVICE_UUID)

        location_characteristic = next(
            c for c in device_information_service.characteristics
            if c.uuid == DWM_LOCATION_CHARACTERISTIC_ID)

        if self.subscribe:
            location_characteristic.enable_notifications()

        location_characteristic.read_value()

    def _decode_position(self, loc, data, off):
        fmt = '<iiib'
        x, y, z, qf = struct.unpack_from(fmt, data, offset=off)
        off += struct.calcsize(fmt)

        loc['calc_pos'] = {
            'x':  float(x) / 1000,
            'y':  float(y) / 1000,
            'z':  float(z) / 1000,
            'qf': qf,
            'valid': True,
        }

        return off

    def _decode_distances(self, loc, data, off):
        fmt = 'b'
        nr_anchors, = struct.unpack_from(fmt, data, offset=off)
        off += struct.calcsize(fmt)

        fmt = '<hib'
        sz = struct.calcsize(fmt)
        i = 0
        while i < nr_anchors and off + sz <= len(data):
            addr, dist, qf = struct.unpack_from(fmt, data, offset=off)
            off += sz
            i += 1

            anchor = {
                'pos': {
                    'qf': qf,
                    'valid': True,
                },
                'dist': {
                    'dist': float(dist) / 1000,
                    'addr': addr,
                    'qf': qf
                },
            }
            loc['anchors'].append(anchor)

        return off

    def _decode_location(self, data):
        fmt = 'b'
        sz = struct.calcsize(fmt)

        if len(data) < sz:
            print("Error: invalid location data received")
            return None

        t, = struct.unpack_from(fmt, data, offset=0)
        off = sz

        loc = {
            'calc_pos': {
                'x':  0.0,
                'y':  0.0,
                'z':  0.0,
                'qf': 0,
                'valid': False,
            },
            'ts': time.time(),
            'anchors': [],
        }

        # See DWM1001-API-Guide.pdf, 7.1.3 notes
        #    t == 0 - position
        #    t == 1 - distances
        #    t == 2 - position + distances
        #
        if t == 1:
            # Distances
            self._decode_distances(loc, data, off)
        elif t == 2:
            # Position + Distances
            off = self._decode_position(loc, data, off)
            self._decode_distances(loc, data, off)
        else:
            print("Error: type %d is not expected" % t)
            return None

        return loc


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

    manager = DwmDeviceManager(adapter_name=args.adapter)
    device = DwmDevice(mac_address=args.mac, manager=manager)

    device.connect()
    manager.start()

    ts = time.time()

    while True:
        r, w, e = select.select([device.eventfd], [], [])
        loc = device.get_location()

        now = time.time()
        rate = 1 / (now - ts)
        ts = now

        print("X = {0}m , Y = {1}m, Z = {2}m,  Quality= {3}, mac={4} rate={5}".format(
            loc['calc_pos']['x'],
            loc['calc_pos']['y'],
            loc['calc_pos']['z'],
            loc['calc_pos']['qf'],
            device.mac_address,
            rate))
