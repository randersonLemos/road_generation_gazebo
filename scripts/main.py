#!/usr/bin/env python

import sys
import numpy as np
import road_generation_gazebo.roads as roads
import road_generation_gazebo.worlds as worlds
import road_generation_gazebo.utils as utils
import road_generation_gazebo.generates as generates

def main( roadpts, roadclose, offsetroad, offsetnpts, offsetvar
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
    #rd.plot_offset_road()
    if withnoise : rd.plot_noise_offset_road()
    else : rd.plot_offset_road()

  wds = worlds.World(
                      model=modelname
                     ,poses=pposes
                    )

  wds.make_custom_world()


if __name__ == '__main__':

  main(
        roadpts=generates.ellipse()
        #roadpts=generates.circle()
        #roadpts=generates.sine_wave()
       ,roadclose=True
       ,offsetroad=2.5
       ,offsetnpts=30
       ,offsetvar=0.25
       ,withnoise=True
       ,withplot=True
       ,modelname='bush'
       #,modelname='cylinder_custom'
      )
