#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

import serial, time, sys, os, struct

def wait_for_string(ser, s, timeout=1.0, debug=False):
    t0 = time.time()
    buf = []
    res = []
    n = 0
    while (True):
        c = ser.read()
        if debug:
            sys.stderr.write(c)
        buf.append(c)
        if len(buf) > len(s):
            res.append(buf.pop(0))
            n += 1
            if n % 10000 == 0:
                sys.stderr.write(str(n) + "\n")
        if "".join(buf) == s:
            break
        if timeout > 0.0 and time.time() - t0 > timeout:
            raise Exception("Timeout while waiting for: " + s)
    return "".join(res)

def send_null(ser):
    null_sh_input = struct.pack('BBB', 0x0, 0x0, 0x0)
    ser.write(null_sh_input)
    ser.flush()

def exec_cmd_without_confirm(ser, cmd):
    ser.write(cmd + "\n")
    ser.flush()

def exec_cmd(ser, cmd, timeout):
    exec_cmd_without_confirm(ser, cmd)
    wait_for_string(ser, cmd + "\r\n", timeout)
    return wait_for_string(ser, "nsh> \x1b[K", timeout)

def has_prompt(ser):
    exec_cmd_without_confirm(ser, "\r\n") # trigger prompt to be sent
    return wait_for_string(ser, "nsh> \x1b[K", 5)

def start_mavlink(ser):
    start_mavlink_cmd = 'sh /etc/init.d/rc.usb'
    exec_cmd_without_confirm(ser, start_mavlink_cmd)
    send_null(ser)
    return True

def exit_terminal(ser):
    return exec_cmd_without_confirm(ser, "exit")

    
def main():
    dev = "/dev/tty.usbmodem1"
    ser = serial.Serial(dev, "57600", timeout=0.2)
    try:
        if has_prompt(ser):
            mavlink_started = start_mavlink(ser)
        else:
            print "No nsh prompt to execute command."
    finally:
        ser.close()

if __name__ == "__main__":
    main()
