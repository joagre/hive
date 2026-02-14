#!/usr/bin/env python3
"""Repair corrupted 1-wire EEPROM on Crazyflie Micro SD Card Deck.

Writes the correct board name ('bcUSD') and revision to the deck's
1-wire memory so the Crazyflie firmware can detect it.

Requires: Bitcraze firmware running on the Crazyflie, cflib installed.
Usage: python3 repair_sd_deck_ow.py [--uri radio://0/80/2M]
"""
import argparse
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import OWElement

logging.basicConfig(level=logging.ERROR)


class RepairOW:
    def __init__(self, uri):
        self._cf = Crazyflie()
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self.is_connected = True
        self.should_disconnect = False

        print(f'Connecting to {uri}')
        self._cf.open_link(uri)

    def _connected(self, link_uri):
        print(f'Connected to {link_uri}')

        mems = self._cf.mem.get_mems(MemoryElement.TYPE_1W)
        print(f'Found {len(mems)} 1-wire memories')

        if len(mems) == 0:
            print('ERROR: No 1-wire memories found. Is the deck connected?')
            self.should_disconnect = True
            return

        # Find the SD card deck (or use the first non-flow deck)
        target = None
        for mem in mems:
            print(f'  Memory {mem.id}: name="{mem.name}" '
                  f'vid=0x{mem.vid:02X} pid=0x{mem.pid:02X} '
                  f'valid={mem.valid}')
            if mem.name == 'bcUSD':
                target = mem
                break
            if not mem.valid or mem.name == '':
                # Corrupted memory - likely our target
                if target is None:
                    target = mem

        if target is None:
            print('No corrupted or SD card deck memory found.')
            print('All memories appear valid. Deck may not be connected.')
            self.should_disconnect = True
            return

        print(f'\nRepairing memory {target.id} '
              f'(current name="{target.name}")...')

        # VID:PID 0x00:0x00 = match by board name
        target.vid = 0x00
        target.pid = 0x00

        board_name_id = OWElement.element_mapping[1]
        board_rev_id = OWElement.element_mapping[2]

        target.elements[board_name_id] = 'bcUSD'
        target.elements[board_rev_id] = 'D'

        target.write_data(self._data_written)

    def _data_written(self, mem, addr):
        print('Write complete, reading back...')
        mem.update(self._data_updated)

    def _data_updated(self, mem):
        print(f'Verified memory {mem.id}:')
        print(f'  Name  : {mem.name}')
        print(f'  VID   : 0x{mem.vid:02X}')
        print(f'  PID   : 0x{mem.pid:02X}')
        print(f'  Valid : {mem.valid}')
        for key in mem.elements:
            print(f'  {key}={mem.elements[key]}')

        if mem.name == 'bcUSD':
            print('\nRepair successful! Power cycle the Crazyflie.')
        else:
            print('\nWARNING: Name mismatch after write.')

        self.should_disconnect = True

    def _connection_failed(self, link_uri, msg):
        print(f'Connection to {link_uri} failed: {msg}')
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print(f'Connection to {link_uri} lost: {msg}')

    def _disconnected(self, link_uri):
        print(f'Disconnected from {link_uri}')
        self.is_connected = False


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Repair SD deck 1-wire EEPROM')
    parser.add_argument('--uri', default='radio://0/80/2M',
                        help='Crazyflie URI (default: radio://0/80/2M)')
    args = parser.parse_args()

    cflib.crtp.init_drivers()
    repair = RepairOW(args.uri)

    try:
        while repair.is_connected:
            if repair.should_disconnect:
                repair._cf.close_link()
            time.sleep(1)
    except KeyboardInterrupt:
        sys.exit(1)
