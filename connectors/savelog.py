#!/usr/bin/env python
# encoding: utf-8

import struct
from twisted.internet.protocol import DatagramProtocol

class XPlaneProtocol(DatagramProtocol):
    def __init__(self, set_state_callback):
        self.f = open('xplane.log', 'wb')

    def datagramReceived(self, data, (host, port)):
        self.dump(data)

    def dump(self, data):
        self.f.write(struct.pack('<i', len(data)))
        self.f.write(bytes(data))
        print "Dumped a packet"

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
