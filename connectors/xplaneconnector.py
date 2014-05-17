from simconnector import SimulatorConnector
from twisted.internet import reactor
from threading import Thread
from xplane import XPlaneProtocol

class XPlaneConnector(SimulatorConnector):

    def __init__(self, hil_callback, sim_ip, sim_port):
        super(XPlaneConnector, self).__init__(hil_callback)
        self.sim_ip = sim_ip
        self.sim_port = sim_port
        self.bind_port = 49005

    def connect(self):
        reactor.listenUDP(self.bind_port, XPlaneProtocol(self.set_simulation_state))
        self.thread = Thread(target=reactor.run,
                             kwargs={'installSignalHandlers' : False})
        self.thread.start()

    def disconnect(self):
        reactor.callFromThread(reactor.stop)
        self.thread.join(10)
