from state import HILStateQuaternion

class SimulatorConnector(object):
    """Connects to a flight data model simulator."""

    def __init__(self, hil_callback):
        self.hil_callback = hil_callback

    def set_simulation_state(self, state):
        self.hil_callback(state)

    def set_controls(self, pitch, roll, yaw, throttle):
        pass
