import sys
import numpy as np
import classes.roads as roads
import classes.worlds as worlds
import others.utils as utils
import others.generates as generates


def main( road_pts, road_isclose, offset
         ,offset_npts, offset_var, doplot
         ,model, fname):

  rd = roads.Road( pts=road_pts
                  ,close=1 if road_isclose else 0
  )

  rd.offset_road( offset=offset
                 ,npts=offset_npts
  )

  rd.noise_offset_road( var=offset_var
  )

  if offset_var:
      poses=utils.position_to_pose(rd.get_noise_offset_road_points())
  else:
      poses=utils.position_to_pose(rd.get_offset_road_points())

  wds = worlds.World( model=model
                     ,poses=poses
                     ,fname = fname
  )

  wds.make_custom_world()

  if doplot:
    rd.plot_road()
    rd.plot_offset_road()
    if offset_var:
        rd.plot_noise_offset_road()
    rd.plot_show()


if __name__ == '__main__':
  main( road_pts=generates.ellipse()
        #road_pts=generates.circle()
        #road_pts=generates.sine_wave()
       ,road_isclose=True
       ,offset=2.5
       ,offset_npts=30
       ,offset_var=1.00
       ,doplot=True
       ,model='bush'
       #,name='cylinder_custom'
       ,fname = 'custom.world'
  )

