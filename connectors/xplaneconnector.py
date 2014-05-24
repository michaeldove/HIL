from simconnector import SimulatorConnector
from twisted.internet import reactor
from threading import Thread
from xplane import XPlaneProtocol

class XPlaneConnector(SimulatorConnector):

    def __init__(self, hil_callback, sim_ip, sim_port=49000, bind_port=49005):
        super(XPlaneConnector, self).__init__(hil_callback)
        self.sim_ip = sim_ip
        self.sim_port = sim_port
        self.bind_port = bind_port
        self.protocol = None
        self.thread = None

    def set_controls(self, roll, pitch, yaw, throttle):
        print "set controls"
        reactor.callFromThread(self.protocol.set_controls, roll, -pitch, yaw, throttle)

    def connect(self):
        print "Connect"
        self.protocol = XPlaneProtocol(self.set_simulation_state)
        reactor.listenUDP(self.bind_port, self.protocol)
        self.thread = Thread(target=reactor.run,
                             kwargs={'installSignalHandlers' : False})
        self.thread.start()

    def disconnect(self):
        if self.thread:
            reactor.callFromThread(reactor.stop)
            self.thread.join(10)
            self.protocol = None
