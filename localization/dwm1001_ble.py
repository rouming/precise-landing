#!/usr/bin/env python3

import dbus
import gatt
import struct
import threading
import time
import eventfd
import signal
import sys
from file_to_sock import send_dwm_data, create_dwm_sock
import config as cfg

DWM_SERVICE_UUID        = '680c21d9-c946-4c1f-9c11-baa1c21329e7'
DWM_LOCATION_DATA_UUID  = '003bbdf2-c634-4b3d-ab56-7ec889b89a37'
DWM_NETWORK_PAN_UUID    = '80f9d8bc-3bff-45bb-a181-2d6a37991208'
DWM_OPERATION_MODE_UUID = '3f0afd88-7770-46b0-b5e7-9fc099598964'
DWM_DEVICE_INFO_UUID    = '1e63b1eb-d4ed-444e-af54-c1e965192501'

class ManagerThread(threading.Thread):
    def __init__(self, manager):
        super().__init__()
        self.manager = manager

    def run(self):
        self.manager.start_discovery()
        # Jumps to manager event loop
        self.manager.run()
        self.manager.stop_discovery()
        self.manager.disconnect_devices()

class DWMDeviceManager(gatt.DeviceManager):
    devs_by_mac_addr = {}
    devs_by_node_addr = {}
    predefined_anchors = {}
    eventfd_map = {}
    inv_eventfd_map = {}
    lock = None

    def __init__(self, adapter_name='hci0', predefined_anchors={},
                 eventfd_map={}):
        self.thread = ManagerThread(self)
        self.lock = threading.Lock()
        self.predefined_anchors = predefined_anchors
        self.eventfd_map = eventfd_map
        # Invert map for efd search
        self.inv_eventfd_map = {v:k for k,v in eventfd_map.items()}
        super(DWMDeviceManager, self).__init__(adapter_name=adapter_name)

    def start(self):
        self.thread.start()

    def stop(self):
        super().stop()
        self.thread.join()

    def device_disconnected(self, device):
        try:
            del self.devs_by_mac_addr[device.mac_address]
        except:
            pass

        try:
            del self.devs_by_node_addr[device.node_addr]
        except:
            pass

    def add_device_by_node_addr(self, device):
        self.lock.acquire()
        self.devs_by_node_addr[device.node_addr] = device
        if device.node_addr in self.eventfd_map:
            # Override the eventfd
            device.eventfd = self.eventfd_map[device.node_addr]
        self.lock.release()

    def find_device_by_node_addr(self, addr):
        device = None
        self.lock.acquire()
        if addr in self.devs_by_node_addr:
            device = self.devs_by_node_addr[addr]
        self.lock.release()

        return device

    def find_device_by_efd(self, efd):
        if efd not in self.inv_eventfd_map:
            return None
        addr = self.inv_eventfd_map[efd]
        return self.find_device_by_node_addr(addr)

    def disconnect_devices(self):
        for device in self.devs_by_mac_addr.values():
            device.disconnect()
        self.devs_by_mac_addr = {}
        self.devs_by_node_addr = {}

    def make_device(self, mac_address):
        return DWMDevice(mac_address=mac_address, manager=self)

    def device_discovered(self, device):
        try:
            if device.mac_address in self.devs_by_mac_addr:
                # Already connected
                return

            service_data = device._properties.Get('org.bluez.Device1', 'ServiceData')
            # Unfortunately SetDiscoveryFilter does not work for DWM devices
            # So check every discovered device
            if DWM_SERVICE_UUID not in service_data:
                return

            addr = device.name_to_dwm_addr()
            if addr not in self.eventfd_map:
                # We connect only to DWM tags
                return

            device.connect()
            self.devs_by_mac_addr[device.mac_address] = device
            print("[%s] %s discovered" % (device.mac_address, device.alias()))
        except:
            pass

class DWMDevice(gatt.Device):
    is_anchor = False
    pan_id    = 0x0000
    node_addr = 0x0000
    location  = None

    def __init__(self, mac_address, manager):
        self.lock = threading.Lock()
        self.eventfd = eventfd.EventFD()
        super(DWMDevice, self).__init__(mac_address=mac_address, manager=manager)

    def name_to_dwm_addr(self):
        dev_alias = self.alias()
        return int(dev_alias.replace('DW', ''), 16)

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        self.manager.device_disconnected(self)

    def services_resolved(self):
        super().services_resolved()

        device_information_service = next(
            s for s in self.services
            if s.uuid == DWM_SERVICE_UUID)

        location_data_characteristic = next(
            c for c in device_information_service.characteristics
            if c.uuid == DWM_LOCATION_DATA_UUID)

        operation_mode_characteristic = next(
            c for c in device_information_service.characteristics
            if c.uuid == DWM_OPERATION_MODE_UUID)

        network_pan_characteristic = next(
            c for c in device_information_service.characteristics
            if c.uuid == DWM_NETWORK_PAN_UUID)

        device_info_characteristic = next(
            c for c in device_information_service.characteristics
            if c.uuid == DWM_DEVICE_INFO_UUID)

        operation_mode_data = operation_mode_characteristic.read_value()
        self._decode_operation_mode(operation_mode_data)

        network_pan_data = network_pan_characteristic.read_value()
        self._decode_network_pan_id(network_pan_data)

        device_info_data = device_info_characteristic.read_value()
        self._decode_device_info(device_info_data)

        location_data = location_data_characteristic.read_value()
        location = self._decode_location_data(location_data)
        if not self.is_anchor:
            self.location = location

        # Enable notifications for location data
        # We need this even for an anchor to keep device connected
        location_data_characteristic.enable_notifications()

        # Account this device
        self.manager.add_device_by_node_addr(self)

    def get_location(self):
        self.lock.acquire()
        self.eventfd.clear()
        location = self.location
        self.location = None
        self.lock.release()
        return location

    def _decode_operation_mode(self, data):
        if not data or len(data) != 2:
            print("Error: incorrect operation mode buffer length")
            return False

        data = bytearray(data)

        #
        # See DWM1001-API-Guide.pdf, 7.1.2 notes
        #

        fmt = '<H'
        mode, = struct.unpack(fmt, data)

        self.is_anchor = not not (mode & (0x1 << 7))
        return True

    def _decode_network_pan_id(self, data):
        if not data or len(data) != 2:
            print("Error: incorrect network pan buffer length")
            return False

        data = bytearray(data)

        fmt = '<H'
        self.pan_id, = struct.unpack(fmt, data)
        return True

    def _decode_device_info(self, data):
        if not data or len(data) != 29:
            print("Error: incorrect device info buffer length")
            return False

        data = bytearray(data)

        fmt = '<HHIIIIIIB'
        self.node_addr, *other = struct.unpack(fmt, data)
        return True

    def _decode_position(self, loc, data, off):
        fmt = '<iiib'
        x, y, z, qf = struct.unpack_from(fmt, data, offset=off)
        off += struct.calcsize(fmt)

        loc['pos'] = {
            'coords':  [float(x) / 1000,
                        float(y) / 1000,
                        float(z) / 1000 ],
            'qf': qf,
            'valid': 1,
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

            # Find position of the anchor by the node address
            valid = 1
            coords = [0.0, 0.0, 0.0]
            if addr in self.manager.predefined_anchors:
                coords = self.manager.predefined_anchors[addr]
            elif addr in self.manager.devs_by_node_addr:
                device = self.manager.devs_by_node_addr[addr]
                if not device.is_anchor:
                    print("Error: device 0x%x is not an anchor!" % device.node_addr)
                    return None

                coords = device.location['pos']['coords']
            else:
                valid = 0

            anchor = {
                'addr': addr,
                'pos': {
                    'coords': coords,
                    'qf': qf,
                    'valid': valid,
                },
                'dist': {
                    'dist': float(dist) / 1000,
                    'qf': qf
                },
            }
            loc['anchors'].append(anchor)

        return off

    def _decode_location_data(self, data):
        fmt = 'b'
        sz = struct.calcsize(fmt)

        if not data or len(data) < sz:
            print("Error: invalid location data received")
            return None

        data = bytearray(data)

        t, = struct.unpack_from(fmt, data, offset=0)
        off = sz

        loc = {
            'pos': {
                'coords': [0.0, 0.0, 0.0],
                'qf': 0,
                'valid': 0,
            },
            'ts': time.time(),
            'anchors': [],
        }

        # See DWM1001-API-Guide.pdf, 7.1.3 notes
        #    t == 0 - position
        #    t == 1 - distances
        #    t == 2 - position + distances
        #
        if t == 0:
            # Position
            self._decode_position(loc, data, off)
        elif t == 1:
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

    def characteristic_value_updated(self, characteristic, value):
        if characteristic.uuid == DWM_LOCATION_DATA_UUID:
            if not self.is_anchor:
                # Only for TAG
                location = self._decode_location_data(value)
                self.lock.acquire()
                self.location = location
                self.eventfd.set()
                self.lock.release()


help = \
"""DWM1001 BLE

Usage:
  dwm1001_ble.py [--hci-dev <dev>] --tag-addr <addr>... [--stream-to-sock]

Options:
  -h --help              Show this screen
  --hci-dev <dev>        Bluetooth device, 'hci0' by default
  --tag-addr <addr>...   Tag node address in hex
  --stream-to-sock       Stream DWM location to sock
"""

from docopt import docopt
import select

should_stop = False
stop_efd = eventfd.EventFD()

def coords_to_mm(pos):
    pos['coords'] = [int(v * 1000) for v in pos['coords']]

def coords_and_dist_to_mm(loc):
    coords_to_mm(loc['pos'])
    for anchor in loc['anchors']:
        coords_to_mm(anchor['pos'])
        anchor['dist']['dist'] = int(anchor['dist']['dist'] * 1000)

def signal_handler(sig, frame):
    print(' You pressed Ctrl+C! Disconnecting all devices ...')
    global should_stop, stop_efd
    should_stop = True
    stop_efd.set()

if __name__ == '__main__':
    args = docopt(help)
    signal.signal(signal.SIGINT, signal_handler)

    tag_addrs = [int(tag_addr, base=16) for tag_addr in args['--tag-addr']]
    tag = None
    ts = time.time()

    hci_dev = 'hci0'
    if args['--hci-dev']:
        hci_dev = args['--hci-dev']

    eventfd_map = {addr: eventfd.EventFD() for addr in tag_addrs}
    manager = DWMDeviceManager(adapter_name=hci_dev,
                               predefined_anchors=cfg.ANCHORS,
                               eventfd_map=eventfd_map)
    manager.start()

    if args['--stream-to-sock']:
        dwm_sock = create_dwm_sock()

    efds = eventfd_map.values()

    while not should_stop:
        r, w, e = select.select([*efds, stop_efd], [], [])
        if should_stop:
            break

        for efd in r:
            tag = manager.find_device_by_efd(efd)
            if not tag:
                print("Error: can't find tag, stopping everything")
                should_stop = True
                break

            now = time.time()
            rate = 1 / (now - ts)
            ts = now
            loc = tag.get_location()

            if args['--stream-to-sock']:
                # We pass through sock integers, not floats
                coords_and_dist_to_mm(loc)
                send_dwm_data(dwm_sock, loc)

            print("[%04x] XYZ = (%.2fm %.2fm %.2fm) %3.0fHz" % \
                  (tag.node_addr,
                   loc['pos']['coords'][0],
                   loc['pos']['coords'][1],
                   loc['pos']['coords'][2],
                   rate))

    manager.stop()
