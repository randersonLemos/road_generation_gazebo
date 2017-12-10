#!/usr/bin/env python

import sys
import rospy
import numpy as np
import road_generation_gazebo.roads as roads
import road_generation_gazebo.worlds as worlds
import road_generation_gazebo.utils as utils
import road_generation_gazebo.generates as generates

def main(road_pts, road_close, offset
        ,offset_npts, offset_var, doplot
        ,model, file_name):

  rd = roads.Road(pts=road_pts
                 ,close=road_close
  )

  rd.offset_road(offset=offset
                ,npts=offset_npts
  )

  rd.noise_offset_road(var=offset_var
  )

  poses=utils.position_to_pose(rd.get_noise_offset_road_points()
  )

  wds = worlds.World(model=model
                    ,poses=poses
                    ,file_name = file_name
  )

  wds.make_custom_world(
  )

  if doplot:
    rd.plot_road()
    rd.plot_offset_road()
    if offset_var:
        rd.plot_noise_offset_road()
    rd.plot_show()


if __name__ == '__main__':

  # Initialize the node and name it.
  rospy.init_node('world_creation', anonymous=True, disable_signals=True)

  # Getting parameters
  metadata = rospy.get_param('~metadata')
  road_close=metadata['close']
  if metadata['mode'] == 'function':
    if metadata['geometry'] == 'ellipse':
      road_pts = generates.ellipse(metadata['radius']
                                  ,metadata['RADIUS']
      )
  elif metadata['mode'] == 'waypoints':
      road_pts = metadata['points']
  else:
    raise ValueError('Not expected mode...')

  main(road_pts=road_pts
      ,road_close=road_close
      ,offset=rospy.get_param('~offset')
      ,offset_npts=rospy.get_param('~offset_npts')
      ,offset_var=rospy.get_param('~offset_var')
      ,doplot=rospy.get_param('~doplot')
      ,model=rospy.get_param('~model')
      ,file_name =rospy.get_param('~file_name')
  )

