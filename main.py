import numpy as np
import classes.roads as roads

x = np.arange(0, 2*np.pi+np.pi/4, 2*np.pi/8)
y = np.sin(x)

#pts = [(-2,-2),(0,-2),(2,-2),(2,0),(2,2),(0,2),(-2,2),(-2,0),(-2,-2)]
pts = zip(x,y)

rd = roads.Road(pts, close=0)
rd.offset_road(0.50, npts=100)
rd.noise_offset_road(var=0.10)
#rd.plot_offset_road()
#rd.plot_noise_offset_road()
