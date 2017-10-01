import numpy as np
import classes.utils as utils
import classes.roads as roads
import classes.worlds as worlds

def generates_sine_wave(rev=5, stretch_factor=0.30, amplification_factor=1):
  x = np.arange(0, 2*np.pi*rev+np.pi/16, np.pi/16)
  y = amplification_factor*np.sin(stretch_factor*x)
  return list(zip(x,y))

def generates_circle(radius=10, begin_angle=0.0, end_angle=2*np.pi):
  th = np.arange(begin_angle, end_angle+np.pi/16, np.pi/16)
  x = radius*np.cos(th)
  y = radius*np.sin(th)
  return list(zip(x,y))

def generates_circle_sine_wave(radius=10, begin_angle=0.0, end_angle=2*np.pi):
  th = np.arange(begin_angle, end_angle+np.pi/16, np.pi/16)
  r = radius + 0.1*radius*np.sin(8*th)
  x = r*np.cos(th)
  y = r*np.sin(th)
  return list(zip(x,y))

if __name__ == '__main__':

  rd = roads.Road(
                   #generates_sine_wave()
                   generates_circle_sine_wave()
                  ,close=1
                 )

  rd.offset_road(2.50, npts=100)
  rd.noise_offset_road(var=0.25)
  rd.plot_offset_road()

  wds = worlds.World(
                     poses=utils.position_to_pose(rd.get_offset_road_points())
                    )

  wds.make_custom_world()
