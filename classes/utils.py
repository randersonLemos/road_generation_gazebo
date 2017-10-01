import math

EMPTY_WORLD = """\
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>\
"""

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

def position_to_pose(pos):
  lst = []
  for p in pos:
    lst.append(p + (0.0,0.0,0.0,0.0))
  return lst

def include_model(model, pose):
  return """\
    <include>
      <uri>model://{}</uri>
      <pose>{} {} {} {} {} {}</pose>
    </include>""".format( model
                         ,pose[0], pose[1], pose[2]
                         ,pose[3], pose[4], pose[5])

