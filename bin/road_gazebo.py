#!/usr/bin/env python

import sys
import rospy
import numpy as np
import road_generation_gazebo.roads as roads
import road_generation_gazebo.worlds as worlds
import road_generation_gazebo.utils as utils
import road_generation_gazebo.generates as generates

def main( road_pts, road_isclose, offset
         ,offset_npts, offset_var, doplot
         ,model, file_name):

  rd = roads.Road( pts=road_pts
                  ,close=1 if road_isclose else 0
  )

  rd.offset_road( offset=offset
                 ,npts=offset_npts
  )

  rd.noise_offset_road( var=offset_var
  )

  poses=utils.position_to_pose(rd.get_noise_offset_road_points())

  wds = worlds.World( model=model
                     ,poses=poses
                     ,file_name = file_name
  )

  wds.make_custom_world()

  if doplot:
    rd.plot_road()
    rd.plot_offset_road()
    if offset_var:
        rd.plot_noise_offset_road()
    rd.plot_show()


if __name__ == '__main__':

  # Initialize the node and name it.
  rospy.init_node('world_creation', anonymous = True)

  # Getting parameters
  metadata = rospy.get_param('~metadata')
  road_isclose=metadata['isclose']
  if metadata['mode'] == 'function':
    if metadata['geometry'] == 'ellipse':
      road_pts = generates.ellipse(
         metadata['radius']
        ,metadata['RADIUS']
        )
  elif metadata['mode'] == 'waypoints':
    raise NotImplementedError('Needs to be implemented...')
  else:
    raise ValueError('Not expected mode...')

  main( road_pts=road_pts
       ,road_isclose=road_isclose
       ,offset=rospy.get_param('~offset')
       ,offset_npts=rospy.get_param('~offset_npts')
       ,offset_var=rospy.get_param('~offset_var')
       ,doplot=rospy.get_param('~doplot')
       ,model=rospy.get_param('~model')
       ,file_name =rospy.get_param('~file_name')
  )
