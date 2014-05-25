#!/usr/bin/env python

'''
runs hil simulation
'''

# system import
import sys, struct, time, os, argparse, signal, math, errno, psutil
import pexpect, socket, fdpexpect, select
import pymavlink.mavutil as mavutil

from pymavlink.dialects.v10 import pixhawk as mavlink
mavutil.set_dialect('pixhawk')

# set path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'modules'))

import hangar
import gcs
from constants import *

# local imports
import util, atexit

from math import sin, cos

class SensorHIL(object):
    ''' This class executes sensor level hil communication '''

    def __init__(self, master_dev, baudrate):
        ''' default ctor 
        @param dev device
        @param baud baudrate
        '''
        self.attack = None
        self.ac = hangar.BasicAircraft(self.attack)
        self.t_hil_state = 0

        self.master = None

        self.counts = {}
        self.bytes_sent = 0
        self.bytes_recv = 0
        self.frame_count = 0
        self.last_report = 0

        self.init_mavlink(master_dev, baudrate)


    def __del__(self):
        print 'SensorHil shutting down'

    def init_mavlink(self, master_dev, baudrate):
        # master
        master = mavutil.mavserial(master_dev, baud=baudrate, autoreconnect=True)
        print 'master connected on device: ', master_dev

        # class data
        self.master = master

    def get_mode_flag(self, flag):
        if (self.master.base_mode & flag) == 0:
            return False
        else:
            return True

    def set_mode_flag(self, flag, enable):
        t_start = time.time()
        if self.get_mode_flag(flag) == enable:
            return
        while not self.get_mode_flag(flag) == enable:
            self.master.set_mode_flag(flag, enable)
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
            time.sleep(0.1)
            if time.time()  - t_start > 5: raise IOError('Failed to set mode flag, check port')

    
    def wait_for_no_msg(self, msg, period, timeout, callback=None):
        done = False
        t_start = time.time()
        t_last = time.time()
        while not done:
            if callback is not None: callback()
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
                if m is None: continue
                if m.get_type() == msg:
                    t_last = time.time()
            if time.time() - t_last > period:
                done = True
            elif time.time() - t_start > timeout:
                done = False
                break
            time.sleep(0.001)

        return done
 
    def wait_for_msg(self, msg, timeout, callback=None):
        done = False
        t_start = time.time()
        while not done:
            if callback is not None: callback()
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
                if m is None: continue
                if m.get_type() == msg:
                    done = True
                    break
            if time.time() - t_start > timeout:
                done = False
                break
            time.sleep(0.1)

        return done

    def reboot_autopilot(self):

        reboot_successful = False
        while not reboot_successful:

            # Request reboot until no heartbeat received
                # The callback option cannot be used for this, because it
                # runs at the same speed as the message receive which is
                # unecessary.
            shutdown = False
            while not shutdown:
                self.set_hil_and_arm()
                #self.set_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED, True)
                print 'Sending reboot to autopilot'
                self.master.reboot_autopilot()
                time.sleep(10)
                #self.master.close()

                #self.master = mavutil.mavserial(self.master.device, baud=self.master.baud, autoreconnect=True)
                # wait for heartbeat timeout, continue looping if not received
                #shutdown = self.wait_for_no_msg(msg='HEARTBEAT', period=2, timeout=10)
            print 'Autopilot heartbeat lost (rebooting)'
            # Try to read heartbeat three times before restarting shutdown
            for i in range(3):
                print 'Attempt %d to read autopilot heartbeat.' % (i+1)
                # Reset serial comm
                self.master.reset()
                reboot_successful = self.wait_for_msg('HEARTBEAT', timeout=100)
                if reboot_successful:
                    #delay sending data to avoid boot problem on px4
                    time.sleep(1)
                    self.set_hil_and_arm()
                    #self.set_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED, True)
                    break

    def wait_for_heartbeat(self):
        """Wait for a heatbeat."""
        heartbeat_received = self.wait_for_msg('HEARTBEAT', timeout=10)
        return heartbeat_received

    def wait_for_mode_flag(self, mode_flag):
        """Wait for a sys status packet then check for mode."""
        sys_status_received = self.wait_for_msg('SYS_STATUS', timeout=10)
        if sys_status_received:
            hil_enabled = self.get_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED)
            return hil_enabled
        return False
        

    def start(self):
        print "Waiting for heartbeat.."
        heartbeat_received = self.wait_for_heartbeat()
        if not heartbeat_received: 
            print "No heartbeat received."
            return

        self.master.mav.command_long_send(1,
                            0,
                            mavlink.MAV_CMD_DO_SET_MODE, 0,
                            225,
                            #mavlink.MAV_MODE_FLAG_HIL_ENABLED|
                            #mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED|
                            #mavlink.MAV_MODE_FLAG_SAFETY_ARMED |
                            #mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            1, 0, 0, 0, 0, 0)

        hil_enabled = self.wait_for_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED)
        if hil_enabled:
            print "HIL Enabled."
        else:
            print "HIL did not enable."
               
        time_start = time.time()
        while time.time() - time_start < 4:
            self.update()
        return time.time()

    @staticmethod
    def interpret_address(addrstr):
        '''interpret a IP:port string'''
        a = addrstr.split(':')
        a[1] = int(a[1])
        return tuple(a)

    def process_master(self):

        m = self.master.recv_msg()
        if m == None: return

        # handle messages
        mtype = m.get_type()

        if mtype == 'HIL_CONTROLS':
            print m.roll_ailerons, m.pitch_elevator, m.yaw_rudder, m.throttle

        elif mtype == 'STATUSTEXT':
            print 'sys %d: %s' % (self.master.target_system, m.text)

    def update(self):
        # receive messages on serial port
        while self.master.port.inWaiting() > 0:
            self.process_master()

    def run(self):
        ''' main execution loop '''

        # start simulation
        t_hil_state = 0

        self.start()

        # run main loop

        while self.update(): pass

    def stop(self):
        '''clean up'''
        pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--device', help='serial device', required=True)
    parser.add_argument('--baud', help='master port baud rate', default=115200)
    args = parser.parse_args()
    if args.device is None:
        raise IOError('must specify device with --device')
    inst = SensorHIL(master_dev=args.device, baudrate=args.baud)
    try:
        inst.run()
    finally:
        inst.stop()

# vim:ts=4:sw=4:expandtab
