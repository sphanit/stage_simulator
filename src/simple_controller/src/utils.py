import math

def normalize_theta(theta):
  PI = math.pi
  result = math.fmod(theta + PI, 2.0 * PI)
  if result <= 0:
    return result + PI
  return result - PI
