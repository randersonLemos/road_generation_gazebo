#!/usr/bin/env python

import sys
import rospy
import numpy as np
import road_generation_gazebo.roads as roads
import road_generation_gazebo.worlds as worlds
import road_generation_gazebo.utils as utils
import road_generation_gazebo.generates as generates

def generation_road( roadpts, roadclose, offsetroad, offsetnpts, offsetvar
         ,withnoise, withplot, modelname):

  rd = roads.Road(
                   pts=roadpts
                  ,close=1 if roadclose else 0
                 )

  rd.offset_road(
                  offset=offsetroad
                 ,npts=offsetnpts
                )

  rd.noise_offset_road(var=offsetvar)

  if withnoise : pposes=utils.position_to_pose(rd.get_noise_offset_road_points())
  else : pposes=utils.position_to_pose(rd.get_offset_road_points())

  if withplot:
    if withnoise : rd.plot_noise_offset_road()
    else : rd.plot_offset_road()

  wds = worlds.World(
                      model=modelname
                     ,poses=pposes
                    )

  wds.make_custom_world()


if __name__ == '__main__':

  # Initialize the node and name it.
  rospy.init_node('world_creation', anonymous = True)

  # Getting parameters
  road_metadata = rospy.get_param('~road_metadata')
  if road_metadata['mode'] == 'function_based':
    road_close=road_metadata['closed']
    if road_metadata['geometry'] == 'ellipse':
      road_waypoints = generates.ellipse(
         road_metadata['r1']
        ,road_metadata['r2']
        )
  elif road_metadata['mode'] == 'user_based':
    raise NotImplementedError('Needs to be implemented...')
  else:
    raise ValueError('Not expected mode...')


  show_plot= rospy.get_param('~show_plot')
  offset_road_add_noise = rospy.get_param('~offset_road_add_noise')
  offset_road_lateral_distance = rospy.get_param('~offset_road_lateral_distance')
  offset_road_linear_density = rospy.get_param('~offset_road_linear_density')
  offset_road_noise_var = rospy.get_param('~offset_road_noise_var')
  model_name = rospy.get_param('~model_name')


  generation_road(
     roadpts=road_waypoints
    ,roadclose=road_close
    ,offsetroad=offset_road_lateral_distance
    ,offsetnpts=offset_road_linear_density
    ,offsetvar=offset_road_noise_var
    ,withnoise=offset_road_add_noise
    ,withplot=show_plot
    ,modelname=model_name
    )
