import time
from xplaneconnector import XPlaneConnector

SIM_IP = "127.0.0.1"
SIM_PORT = 49005

def callback(state):
    print state

connector = XPlaneConnector(callback, SIM_IP, SIM_PORT)
try:
    connector.connect()
    time.sleep(20)
finally:
    connector.disconnect()
