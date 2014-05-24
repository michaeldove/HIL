from math import cos as cosf
from math import sin as sinf
def quaternion_from_euler(roll, pitch, heading):
    initialRoll = roll
    initialPitch = pitch
    initialHdg = heading

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    cosRoll = cosf(initialRoll * 0.5);
    sinRoll = sinf(initialRoll * 0.5);

    cosPitch = cosf(initialPitch * 0.5);
    sinPitch = sinf(initialPitch * 0.5);

    cosHeading = cosf(initialHdg * 0.5);
    sinHeading = sinf(initialHdg * 0.5);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading; #w
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading; #xi
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading; #yj
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading; #zk
    return (q0, q1, q2, q3)

if __name__ == '__main__':
    import math
    print "roll: %s" %  repr(quaternion_from_euler(math.radians(180), 0, 0))
    print "pitch: %s" % repr(quaternion_from_euler(0, math.radians(180), 0))
    print "heading: %s" % repr(quaternion_from_euler(0, 0, math.radians(180)))
