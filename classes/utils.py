import math

def chord_length(pts):
  cl = []
  cl.append(0.0)
  for one, other in zip(pts[1:],pts[:-1]):
    cl.append(math.sqrt((one[0]-other[0])**2+(one[1]-other[1])**2))
  return cl

def sum_chord_length(chord_length):
  scl = []
  scl.append(0)
  for cl in chord_length[1:]:
    scl.append(cl + scl[-1])
  return scl
