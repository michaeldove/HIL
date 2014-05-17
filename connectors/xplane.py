#!/usr/bin/env python
# encoding: utf-8

#import socket
from struct import *
import math
import time
from twisted.internet.protocol import DatagramProtocol
from Quaternion import Quat
from state import HILStateQuaternion

#UDP_IP = "192.168.1.165"
#UDP_SENDTO_PORT=49001 # This is the port X-Plane is listening on for commands (such as the data select packet we send to tell it what data to send us).

SPEED_INDEX = 3
MACH_VVI_GLOAD_INDEX = 4
PITCH_ROLL_HEADINGS_INDEX = 18
LAT_LON_ATTITUDE_INDEX = 20
ANGULAR_VELOCITIES_INDEX = 17
LOC_VEL_DIST_INDEX = 21

START_INDEX_OFFSET = 5
INDEX_LENGTH = 4
FLOAT_SIZE = 4
DATA_ARRAY_LENGTH = 8
DATA_LENGTH = FLOAT_SIZE * DATA_ARRAY_LENGTH
CHUNK_LENGTH = INDEX_LENGTH + DATA_LENGTH

class UnitConversion:

    GRAVITY_ACCEL = 9.81

    @classmethod
    def knots_to_ms(self, knots):
        return knots * 0.514444

    @classmethod
    def feet_to_meters(self, feet):
        return feet * 0.3048

    @classmethod
    def seconds_to_useconds(self, seconds):
        return seconds * 1000000

    @classmethod
    def gravity_to_acceleration(self, gravity):
        return gravity * self.GRAVITY_ACCEL


class XPlaneProtocol(DatagramProtocol):

    def __init__(self, set_state_callback):
        """
        This will send a packet to X-Plane to select the data you need to read.
        Once it's sent X-Plane will output data automatically at a default of 20Hz.
        In this string, "\x03\x00\x00\x00", we are selecting the third checkbox in the
        "Settings" > "Data Input and Output" menu item ("speeds" in this example). And
        don't forget that these numbers are in hexadecimal!
        """
        self.set_state_callback = set_state_callback

        self.parser_map = {
            SPEED_INDEX : self.parse_speed,
            MACH_VVI_GLOAD_INDEX : self.parse_mach_vvi_gload,
            PITCH_ROLL_HEADINGS_INDEX : self.parse_pitch_roll_headings,
            LAT_LON_ATTITUDE_INDEX : self.parse_lat_lon_attitude,
            ANGULAR_VELOCITIES_INDEX : self.parse_angular_velocities,
            LOC_VEL_DIST_INDEX : self.parse_loc_velocity_distance
            }

    def data_select(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP,49999)) # this is the port we're sending from and it doesn't really matter
        data_selection_packet = "DSEL0" # this is the xplane packet type
        data_selection_packet += "\x03\x00\x00\x00" # airspeed
        data_selection_packet += "\x04\x00\x00\x00" # accelerometers
        data_selection_packet += "\x06\x00\x00\x00" # temperature
        data_selection_packet += "\x11\x00\x00\x00" # gyros
        data_selection_packet += "\x12\x00\x00\x00" # pitch and roll (for sanity check)
        data_selection_packet += "\x14\x00\x00\x00" # altimeter and GPS
        self.sock.sendto(data_selection_packet,(UDP_IP,UDP_SENDTO_PORT))

    def datagramReceived(self, data, (host, port)):
        """
        We've received a UDP packet from X-Plane (which is in binary so printing it
        won't help). This is where we unpack and then do whatever with the data.
        """
        state = self.unpack(data)
        if self.set_state_callback and state:
            self.set_state_callback(state)

    def parse_speed(self, payload, state):
        raw_speed = unpack_from('<ffffffff', payload) 
        kias, keas, ktas, ktgs, empty, mph, mphas, mphgs = raw_speed
        state.velocity_ias_ms = UnitConversion.knots_to_ms(kias)
        state.velocity_tas_ms = UnitConversion.knots_to_ms(ktas)

    def parse_mach_vvi_gload(self, payload, state):
        parsed_payload = unpack_from('<ffffffff', payload) 
        normal_g, axial_g, side_g = parsed_payload[4:4+3]
        normal_accel = -UnitConversion.gravity_to_acceleration(normal_g)
        axial_accel = UnitConversion.gravity_to_acceleration(axial_g)
        side_accel = UnitConversion.gravity_to_acceleration(side_g)
        state.xacc = side_accel
        state.yacc = axial_accel
        state.zacc = normal_accel

    def parse_pitch_roll_headings(self, payload, state):
        parsed_payload = unpack_from('<ffffffff', payload) 
        pitch_deg, roll_deg, yaw_deg, yaw_mag_deg = parsed_payload[0:4]
        q = Quat((pitch_deg, roll_deg, yaw_deg)).q
        w, i, j, k = q
        state.attitude.w = w
        state.attitude.i = i
        state.attitude.j = j
        state.attitude.k = k

    def parse_lat_lon_attitude(self, payload, state):
        parsed_payload = unpack_from('<ffffffff', payload) 
        lat, lon, alt_ftmsl, alt_ftagl = parsed_payload[0:4]
        state.lat = lat
        state.lon = lon
        state.alt = UnitConversion.feet_to_meters(alt_ftagl)

    def parse_angular_velocities(self, payload, state):
        parsed_payload = unpack_from('<ffffffff', payload) 
        q, p, r = parsed_payload[0:3]
        state.pitch_speed = q
        state.roll_speed = p
        state.yaw_speed = r
        
    def parse_loc_velocity_distance(self, payload, state):
        parsed_payload = unpack_from('<ffffffff', payload) 
        v_lat, v_alt, v_lon = parsed_payload[3:3+3]
        state.vx = v_lat
        state.vy = v_lon
        state.vz = v_alt

    def has_valid_header(self, data):
        header = unpack_from('<cccc', data, 0)
        return ''.join(header) == 'DATA'

    def unpack(self, data):
        """
        DATA\0|index1|payload1|index2|payload2
              |    CHUNK1     |    CHUNK2
        """
        if not self.has_valid_header(data): return False
        state = HILStateQuaternion()
        state.time_usec = UnitConversion.seconds_to_useconds(time.time())
        for chunk_offset in range(START_INDEX_OFFSET,len(data), CHUNK_LENGTH):
            payload_index = unpack_from('<i', data, chunk_offset)[0]
            payload_offset = chunk_offset+INDEX_LENGTH
            payload = data[payload_offset:payload_offset+DATA_LENGTH]
            parser_func = self.parser_map.get(payload_index)
            if parser_func:
                parser_func(payload, state)
        return state

    def display(self):
        """
        Use the curses library if you want to format and update the data nicely on the
        screen (similar to the "top" command in *nix)
        """
        print "Vair (knots): %f, Vtgs (knots): %f, accelx: %f, accely: %f, accelz: %f, p: %f, q: %f, r: %f, alt (ftmsl): %f, lat: %f, long: %f" % (self.Vair, self.Vtgs, self.ax, self.ay, self.az, self.p, self.q, self.r, self.altitude, self.latitude, self.longitude)

if __name__ == "__main__":
    """
    If you call this file directly (from the command line using "python xplane.py") then
    it will sleep until a packet is received and then call datagramReceived when one arrives.
    """
    print "Listening to X-Plane..."

    from twisted.internet import reactor
    UDP_PORT=49005 # I'm not sure if there's a default so you'll have to set it like this: http://dronedynamics.com/xplane_config.png
    xplane_connector = XPlaneProtocol(None)
    reactor.listenUDP(UDP_PORT, xplane_connector)
    reactor.run()
