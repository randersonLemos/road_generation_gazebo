import utils
import numpy as np
from frozen import FrozenClass
from scipy import interpolate



class Road(FrozenClass):
  """
  Computes a circuit (road) from a list of points.

  Parameters
  ----------
  pts : [..., (xi, yi), ...]
    List of points
  close : 0/1
    0 for open cirucits and 1 for closed ones. Default is 1
  degree : int
    Degree of the spline polynomial. Default is 3
  """
  def __init__(self, pts, close=1, degree=3):
    [self.x, self.y] = zip(*pts)
    self.tck, self.t = interpolate.splprep([self.x, self.y], s=0, per=close, k=degree)


  def show_road(self):
    import matplotlib.pyplot as plt
    t = np.arange(0.0,1.01,0.01)
    out = interpolate.splev(t, self.tck)
    plt.figure
    plt.plot(self.x, self.y, 'o')
    plt.plot(out[0],out[1])
    plt.show()

