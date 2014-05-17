import struct
import sched, time
import socket

DEST_IP = '127.0.0.1'
DEST_PORT = 49005

s = sched.scheduler(time.time, time.sleep)
f = open('xplane.log', 'rb')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def read_packet():
    data = None
    def read_chunk():
        length_bytes = f.read(4)
        length = struct.unpack('<i', length_bytes)[0]
        return f.read(length)

    try:
        data = read_chunk()
#        length_bytes = f.read(4)
#        length = struct.unpack('<i', length_bytes)
#        data = f.read(length)
    except (ValueError, struct.error):
        #end of file or corruption
        f.seek(0)
        data = read_chunk()
        #length = int(f.read(3))
        #data = f.read(length)
    return data


def write_packet(sc): 
    packet = read_packet()
    sock.sendto(packet,(DEST_IP, DEST_PORT))
    sc.enter(0.5, 1, write_packet, (sc,))
    
try:
    s.enter(0, 1, write_packet, (s,))
    s.run()
finally:
    f.close()
    sock.close()
