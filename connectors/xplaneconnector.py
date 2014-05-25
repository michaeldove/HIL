import socket, select
from simconnector import SimulatorConnector
from twisted.internet import reactor
from xplane import XPlaneProtocol

class XPlaneConnector(SimulatorConnector):

    def __init__(self, hil_callback, sim_ip, sim_port=49000, bind_port=49005):
        super(XPlaneConnector, self).__init__(hil_callback)
        self.sim_ip = sim_ip
        self.sim_port = sim_port
        self.bind_port = bind_port
        self.protocol = None

    def set_controls(self, roll, pitch, yaw, throttle):
        packets = self.protocol.encode_controls(roll, pitch, yaw, throttle)
        for packet in packets:
            self.sock.sendto(packet, (self.sim_ip, self.sim_port))

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(0)
        self.sock.bind(('', self.bind_port))
        self.protocol = XPlaneProtocol(self.set_simulation_state)
        return self.sock

    def handle_read(self, sock):
        data = sock.recv(1024)
        self.protocol.datagramReceived(data, (None, None))

    def disconnect(self):
        pass
