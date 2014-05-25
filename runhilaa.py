#!/usr/bin/env python

'''
runs hil simulation
'''

# system import
import sys, struct, time, os, argparse, signal, math, errno, psutil
import pexpect, socket, fdpexpect, select
import pymavlink.mavutil as mavutil

from pymavlink.dialects.v10 import pixhawk as mavlink
#mavutil.set_dialect('pixhawk')

# set path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'modules'))

import hangar
import gcs
from constants import *

# local imports
import util, atexit
import pymavlink.fgFDM as fgFDM
from connectors.xplaneconnector import XPlaneConnector

from math import sin, cos

class SensorHIL(object):
    ''' This class executes sensor level hil communication '''

    def __init__(self, master_dev, baudrate, script, options, gcs_dev, waypoints, mode, fgout):
        ''' default ctor 
        @param dev device
        @param baud baudrate
        '''
        self.script = script
        self.options = options
        self.waypoints = waypoints
        self.mode = mode

        self.attack = None
        self.ac = hangar.BasicAircraft(self.attack)
        self.t_hil_state = 0


        self.gcs = None
        self.master = None

        self.counts = {}
        self.bytes_sent = 0
        self.bytes_recv = 0
        self.frame_count = 0
        self.last_report = 0

        SIM_IP = "10.0.0.1"
        SIM_PORT = 49000
        self.sim_connector = XPlaneConnector(self.process_connector_state,
                                             SIM_IP, SIM_PORT)

        self.init_mavlink(master_dev, gcs_dev, baudrate)
        self.wpm = gcs.WaypointManager(self.master)


    def __del__(self):
        print 'SensorHil shutting down'
        self.sim_connector.disconnect()

    def init_mavlink(self, master_dev, gcs_dev, baudrate):

        # master
        master = mavutil.mavserial(master_dev, baud=baudrate, autoreconnect=True)
        print 'master connected on device: ', master_dev

        # gcs
        if gcs_dev is not None:
            gcs = mavutil.mavudp(gcs_dev, input=False)
            print 'gcs connected on device: ', gcs_dev

        # class data
        self.master = master
        self.gcs = gcs

#    def init_jsbsim(self):
#        cmd = "JSBSim --realtime --suspend --nice --simulation-rate=1000 --logdirectivefile=data/flightgear.xml --script=%s" % self.script
#        if self.options:
#            cmd += ' %s' % self.options
#
#        jsb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
#        jsb.delaybeforesend = 0
#        util.pexpect_autoclose(jsb)
#        i = jsb.expect(["Successfully bound to socket for input on port (\d+)",
#                        "Could not bind to socket for input"])
#        if i == 1:
#            print("Failed to start JSBSim - is another copy running?")
#            sys.exit(1)
#        jsb_out_address = self.interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
#        jsb.expect("Creating UDP socket on port (\d+)")
#        jsb_in_address = self.interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
#        jsb.expect("Successfully connected to socket for output")
#        jsb.expect("JSBSim Execution beginning")
#
#        # setup output to jsbsim
#        print("JSBSim console on %s" % str(jsb_out_address))
#        jsb_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#        jsb_out.connect(jsb_out_address)
#        jsb_console = fdpexpect.fdspawn(jsb_out.fileno(), logfile=sys.stdout)
#        jsb_console.delaybeforesend = 0
#        jsb_console.logfile = None
#
#        # setup input from jsbsim
#        print("JSBSim FG FDM input on %s" % str(jsb_in_address))
#        jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#        jsb_in.bind(jsb_in_address)
#        jsb_in.setblocking(0)
#
#        # set class data
#        self.jsb = jsb
#        self.jsb_in = jsb_in
#        self.jsb_out = jsb_out
#        self.jsb_console = jsb_console
#        self.fdm = fgFDM.fgFDM()
#
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
            #if time.time()  - t_start > 5: raise IOError('Failed to set mode flag, check port')

    def go_autonomous(self):
        t_start = time.time()
        if (self.get_mode_flag(mavlink.MAV_MODE_FLAG_AUTO_ENABLED) and
           self.get_mode_flag(mavlink.MAV_MODE_FLAG_SAFETY_ARMED)):
            return
        while (not
               (self.get_mode_flag(mavlink.MAV_MODE_FLAG_AUTO_ENABLED)
               and 
               self.get_mode_flag(mavlink.MAV_MODE_FLAG_SAFETY_ARMED))):
            print "SENDING AUTO command"
            self.master.mav.command_long_send(self.master.target_system,
                                self.master.target_component,
                                mavlink.MAV_CMD_DO_SET_MODE, 4,
                                mavlink.MAV_MODE_FLAG_AUTO_ENABLED |
                                mavlink.MAV_MODE_FLAG_SAFETY_ARMED |
                                mavlink.MAV_MODE_FLAG_HIL_ENABLED,
                                0, 0, 0, 0, 0, 0)
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
            time.sleep(0.1)
            if time.time()  - t_start > 5: raise IOError('Failed to '\
                    + 'transition to auto mode, check port and firmware')

    def send_hil(self):
        import pdb
        pdb.set_trace()
        self.master.mav.set_mode_send(self.master.target_system, mavlink.MAV_MODE_FLAG_HIL_ENABLED, 0)
#        self.set_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED, True)
#        self.master.mav.command_long_send(self.master.target_system,
#                            self.master.target_component,
#                            mavlink.MAV_CMD_DO_SET_MODE, 0,
#                            mavlink.MAV_MODE_FLAG_HIL_ENABLED,
#                            0, 0, 0, 0, 0, 0)
# 
                    
    
    def set_hil_and_arm(self):
        t_start = time.time()
        hil_enabled = self.get_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED)
        armed = self.get_mode_flag(mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        if hil_enabled and armed: return
        while not (hil_enabled and armed):
            print "SET HIL and ARM"
            #self.set_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED, True)
#            self.master.mav.command_long_send(self.master.target_system,
#                                self.master.target_component,
#                                mavlink.MAV_CMD_DO_SET_MODE, 4,
#                                mavlink.MAV_MODE_FLAG_SAFETY_ARMED |
#                                mavlink.MAV_MODE_FLAG_HIL_ENABLED,
#                                0, 0, 0, 0, 0, 0)
            while self.master.port.inWaiting() > 0:
                m = self.master.recv_msg()
            time.sleep(0.1)
            if time.time()  - t_start > 5: raise IOError('Failed to '\
                    + 'transition to HIL mode and arm, check port and firmware')


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

#    def jsb_set(self, variable, value):
#        '''set a JSBSim variable'''
#        self.jsb_console.send('set %s %s\r\n' % (variable, value))
#
    def start(self):

#        if self.jsb is not None:
#            print 'quitting jsbsim'
#            self.jsb.close(force=True)
#            self.jsb_out.close()
#            self.jsb_in.close()
#            #self.jsb_console.close()
#
        # reset autopilot state
        #self.reboot_autopilot()
        #time.sleep(8)


        #self.send_hil()
        #self.set_hil_and_arm()
        self.sim_in = self.sim_connector.connect()

        #time.sleep(5)

        #self.go_autonomous()


#        self.init_jsbsim()
#
#        # reset jsbsim state and then pause simulation
#        self.jsb_console.send('resume\n')
#        self.jsb_set('simulation/reset',1)
        self.update()
#        self.jsb.expect("\(Trim\) executed")
#        self.jsb_console.send('hold\n')

#        print 'load waypoints'
#        if not self.waypoints is None:
#            self.wpm.set_waypoints(self.waypoints)
#            self.wpm.send_waypoints()
#
#        while self.wpm.state != 'IDLE':
#            self.process_master()
#
        #self.set_mode_flag(mavlink.MAV_MODE_FLAG_HIL_ENABLED, True)
        #self.set_mode_flag(mavlink.MAV_MODE_FLAG_SAFETY_ARMED, True)
        
#        self.jsb_console.send('resume\n')
#        self.process_jsb_input()
#        self.ac.send_state(self.master.mav)
        # send initial data for estimators to work and system to be able to
        # switch to autonomous mode
        #print 'sending sensor data'
        time_start = time.time()
        #while time.time() - time_start < 5:
        while time.time() - time_start < 4:
            self.update()
        # It might seem like a good idea to wait for a little bit here becuase
        # the first call or two to go_autonomous() is temporarily rejected.
        # DON'T DO IT. You will get really unstable behavior and have no
        # idea why.

        # arm and enter autonomous mode
        #self.set_hil_and_arm()
#        self.go_autonomous()
        # resume simulation
        return time.time()

    def process_sim(self):
        self.sim_connector.handle_read(self.sim_in)

    def process_connector_state(self, state):
        quat = [state.attitude.i, state.attitude.j, state.attitude.k, state.attitude.w]
        try:
            self.master.mav.hil_state_quaternion_send(
                state.time_usec, quat,
                state.roll_speed, state.pitch_speed, state.yaw_speed,
                int(state.lat*1.0e7), int(state.lon*1.0e7), int(state.alt*1000),
                int(state.vx*100), int(state.vy*100), int(state.vz*100),
                max(int(state.velocity_ias_ms*100), 0), max(int(state.velocity_tas_ms*100), 0),
                int(state.xacc*1000), int(state.yacc*1000), int(state.zacc*1000))
        except:
            print state
        #self.fdm.parse(state)
        #self.ac.update_state(self.fdm)

        # TODO should send fdm global pos data to gcs here
        #m = mavutil.mavlink.MAVLink_heartbeat_message(0,0,0,0,0,0)
        #self.gcs.write(m.get_msgbuf())

#        if self.fg_enable:
#            try:
#                self.fg_out.send(self.fdm.pack())
#            except socket.error as e:
#                if e.errno not in [errno.ECONNREFUSED]:
#                    raise
#
    @staticmethod
    def interpret_address(addrstr):
        '''interpret a IP:port string'''
        a = addrstr.split(':')
        a[1] = int(a[1])
        return tuple(a)

    def process_master(self):

        # send waypoint messages to mav
        #self.wpm.send_messages()

        m = self.master.recv_msg()
        if m == None: return

        #print m.get_type()

        # forward to gcs
        #self.gcs.write(m.get_msgbuf())

        # record counts
#        if m.get_type() not in self.counts:
#            self.counts[m.get_type()] = 0
#        self.counts[m.get_type()] += 1

        # handle messages
        mtype = m.get_type()

        if mtype == 'BAD_DATA':
            print m.to_dict()
#            import pdb
#            pdb.set_trace()


        if mtype == 'HIL_CONTROLS':
            #print "HIL_CONTROLS"
            #print m.roll_ailerons, m.pitch_elevator, m.yaw_rudder, m.throttle
            self.sim_connector.set_controls(m.roll_ailerons, m.pitch_elevator,
                                            m.yaw_rudder, m.throttle)
            #self.ac.update_controls(m)
            #self.ac.send_controls(self.jsb_console)

        elif mtype == 'STATUSTEXT':
            print 'sys %d: %s' % (self.master.target_system, m.text)

        # handle waypoint messages
#        self.wpm.process_msg(m)

    def process_gcs(self):
        '''process packets from MAVLink slaves, forwarding to the master'''
        try:
            buf = self.gcs.recv()
        except socket.error:
            return
        try:
            if self.gcs.first_byte:
                self.gcs.auto_mavlink_version(buf)
            msgs = self.gcs.mav.parse_buffer(buf)
        except mavutil.mavlink.MAVError as e:
            print "Bad MAVLink gcs message from %s: %s" % (slave.address, e.message)
            return
        if msgs is None:
            return
        for m in msgs:
            self.master.write(m.get_msgbuf())

        if 'Slave' not in self.counts:
            self.counts['Slave'] = 0
        self.counts['Slave'] += 1

    def update(self):
        # watch files
        rin = [self.gcs.fd, self.sim_in]

        # receive messages on serial port
        while self.master.port.inWaiting() > 0:
            self.process_master()

        try:
            (rin, win, xin) = select.select(rin, [], [], 1.0)
        except select.error:
            util.check_parent()
            return

        # if new gcs input, process it
        if self.gcs.fd in rin:
            self.process_gcs()

        if self.sim_in in rin:
            self.process_sim()

        # if new jsbsim input, process it
#        if self.jsb_in.fileno() in rin:
#            if self.mode == 'state':
#                if (time.time() - self.t_hil_state) > 1.0/50:
#                    self.t_hil_state = time.time()
#                    self.ac.send_state(self.master.mav)
#            elif self.mode == 'sensor':
#                self.ac.send_sensors(self.master.mav)
#            self.process_jsb_input()
#            # gcs not currently getting HIL_STATE message
#            #self.x.send_to_mav(self.gcs.mav)
#            self.frame_count += 1
#
        # show any jsbsim console output

        dt_report = time.time() - self.last_report
        if dt_report > 5:
            print '\nmode: {0:X}, JSBSim {1:5.0f} Hz, {2:d} sent, {3:d} received, {4:d} errors, bwin={5:.1f} kB/s, bwout={6:.1f} kB/s'.format(
                self.master.base_mode,
                self.frame_count/dt_report,
                self.master.mav.total_packets_sent,
                self.master.mav.total_packets_received,
                self.master.mav.total_receive_errors,
                0.001*(self.master.mav.total_bytes_received-self.bytes_recv)/dt_report,
                0.001*(self.master.mav.total_bytes_sent-self.bytes_sent)/dt_report)
            print self.counts
            self.bytes_sent = self.master.mav.total_bytes_sent
            self.bytes_recv = self.master.mav.total_bytes_received
            self.frame_count = 0
            self.last_report = time.time()

        return True

    def run(self):
        ''' main execution loop '''

        # start simulation
        t_hil_state = 0

        self.start()

        # run main loop

        while self.update(): pass

    def stop(self):
        '''clean up'''
        self.sim_connector.disconnect()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--device', help='serial device', required=True)
    parser.add_argument('--baud', help='master port baud rate', default=921600)
    parser.add_argument('--script', help='jsbsim script', default='data/easystar_test.xml')
    parser.add_argument('--options', help='jsbsim options', default=None)
    parser.add_argument('--gcs', help='gcs host', default='localhost:14550')
    parser.add_argument('--waypoints', help='waypoints file', default='data/sf_waypoints.txt')
    parser.add_argument('--mode', help="hil mode (sensor or state)", default='sensor')
    parser.add_argument('--fgout', help="flight gear output", default=None)
    args = parser.parse_args()
    if args.device is None:
        raise IOError('must specify device with --device')
    if args.mode not in ['sensor','state']:
        raise IOError('mode must be sensor or state')
    inst = SensorHIL(master_dev=args.device, baudrate=args.baud, script=args.script, options=args.options, gcs_dev=args.gcs, waypoints=args.waypoints, mode = args.mode, fgout=args.fgout)
    try:
        inst.run()
    finally:
        inst.stop()

# vim:ts=4:sw=4:expandtab
