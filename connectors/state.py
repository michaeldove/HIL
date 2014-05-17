class Quaternion:
    def __init__(self, i=0, j=0, k=0, w=1):
        self.i = i
        self.j = j
        self.k = k
        self.w = w

    def __repr__(self):
        return "%fi+%fj+%fk+%fw" % (self.i, self.j, self.k, self.w)

class HILStateQuaternion:

    def __init__(self):
        self.time_usec = 0 #Done
        self.attitude = Quaternion() # Done
        self.roll_speed = 0 #Done
        self.pitch_speed = 0 #Done
        self.yaw_speed = 0 #Done
        self.lat = 0 #Done
        self.lon = 0 #Done
        self.alt = 0 #Done
        self.vx = 0 #Done
        self.vy = 0 #Done
        self.vz = 0 #Done
        self.velocity_ias_ms = 0 # Done
        self.velocity_tas_ms = 0 # Done
        self.xacc = 0 # Done
        self.yacc = 0 # Done
        self.zacc = 0 # Done

    def __repr__(self):
        return "time (usec): %d, roll speed (rad/s): %f, pitch speed (rad/s): %f, yaw speed (rad/s): %f, lat: %f, lon: %f, alt: %d, vx: %d, vy: %d, vz: %d, ias (m/s): %d, tas (m/s) %d, x accel (m/s/s): %f, y accel (m/s/s): %f, z accel (m/s/s): %f attitude: %s" % (self.time_usec, self.roll_speed, self.pitch_speed, self.yaw_speed, self.lat, self.lon, self.alt, self.vx, self.vy, self.vz, self.velocity_ias_ms, self.velocity_tas_ms, self.xacc, self.yacc, self.zacc, self.attitude)
