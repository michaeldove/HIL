# http://www.du.edu/~jcalvert/math/quatern.htm
# http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/minorlogic.htm

# what quaternion will rotate from a direction along the x axis to point to P2?
# let P1=(1,0,0)
# this gives p2 = (1- 2*qz*qz- 2*qy*qy ,2*qx*qy+ 2*qw*qz , 2*qx*qz- 2*qw*qy)

# p2.x = qw*qw + qx*qx - qz*qz - qy*qy
# p2.y = 2*qx*qy+ 2*qw*qz
# p2.z = 2*qx*qz- 2*qw*qy
# 1 = qw*qw + qx*qx + qz*qz + qy*qy

from math import sqrt, fabs

#from support import is_scalar

class Quaternion:

   def __init__(self, a=0.0, x=0.0, y=0.0, z=0.0):
      # scalar component
      self.a = a

      # vector components
      self.x = x
      self.y = y
      self.z = z

   def __copy__(self):
      return Quaternion(self.a, self.x, self.y, self.z)

   copy = __copy__

   def __neg__(self):
      return Quaternion( -self.a, -self.x, -self.y, -self.z )

   # of the form a*q
   def __rmul__(self, scalar):
      return Quaternion( scalar * self.a, scalar*self.x, scalar*self.y, scalar*self.z )

   # of the form a/q
   def __rdiv__(self, scalar):
      return scalar*self.inv()

   # works for scalars and other quaternions
   def __mul__(self, r):
      q = self
      if is_scalar(r):
         return Quaternion( r*q.a, r*q.x, r*q.y, r*q.z )
      else:
         return Quaternion(
            q.a*r.a - q.x*r.x - q.y*r.y - q.z*r.z,
            q.a*r.x + q.x*r.a + q.y*r.z - q.z*r.y,
            q.a*r.y - q.x*r.z + q.y*r.a + q.z*r.x,
            q.a*r.z + q.x*r.y - q.y*r.x + q.z*r.a )

   def __div__(self, r):
      q = self
      if is_scalar(r):
         return Quaternion( q.a/r, q.x/r, q.y/r, q.z/r )
      else:
         return self*r.inv()

   def conj(self):
      return Quaternion(self.a, -self.x, -self.y, -self.z)

   # same as conjugate for unit quaternions. Proof is trivial!
   def inv(self):
      return self.conj() / ( self*self.conj() ).a

   def rotate(self, p):
      r = self*p*self.conj()

      return Vector( r.x, r.y, r.z )

   def magnitude(self):
      s = self
      return sqrt( s.a*s.a + s.x*s.x + s.y*s.y + s.z*s.z )

   def __len__(self):
      return 4

   def __repr__(self):
      return '%.3f %s %.3fi %s %.3fj %s %.3fk' % ( self.a,
         ('+', '-')[self.x<0], fabs(self.x),
         ('+', '-')[self.y<0], fabs(self.y),
         ('+', '-')[self.z<0], fabs(self.z)
      )

   def normalize(self):
      return self / self.magnitude()

class Vector(Quaternion):
   def __init__(self, x=0.0, y=0.0, z=0.0):
      self.x = x
      self.y = y
      self.z = z

      self.a = 0.0

   def AsTuple(self):
      return (self.x, self.y, self.z)

from math import sin, cos

# http://www.sjbrown.co.uk/?article=quaternions

def from_angles(x, y, z):

   cx = cos(x/2.0)
   sx = sin(x/2.0)
   cy = cos(y/2.0)
   sy = sin(y/2.0)
   cz = cos(z/2.0)
   sz = sin(z/2.0)

   return Quaternion( \
      cx*cy*cz + sx*sy*sz,
      sx*cy*cz - cx*sy*sz,
      cx*sy*cz + sx*cy*sz,
      cx*cy*sz - sx*sy*cz
   )

   # equivalent to: (hmm, what's the performance difference?)

   #qx = Quaternion( cos(x/2.0), sin(x/2.0), 0.0, 0.0 )

   #qy = Quaternion( cos(y/2.0), 0.0, sin(y/2.0), 0.0 )

   #qz = Quaternion( cos(z/2.0), 0.0, 0.0, sin(z/2.0) )

   #return qz*qy*qx

UnitQuaternion = Quaternion(1.0, 0.0, 0.0, 0.0)


# http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
# http://forum.onlineconversion.com/showthread.php?t=5408
# google: quaternions rotations euler angles
# http://www.genesis3d.com/~kdtop/Quaternions-UsingToRepresentRotation.htm
