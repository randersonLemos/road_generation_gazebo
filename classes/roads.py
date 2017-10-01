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
    self._figure_on = False


  def road(self, t):
    out = interpolate.splev(t, self.tck)
    return out


  def der_road(self, t, order=1):
    out = interpolate.splev(t, self.tck, der=order)
    return out


  def offset_road(self, offset, npts=100):
    self.size = npts
    t = np.arange(self.t[0], self.t[-1]+1.0/npts, 1.0/npts)
    [x,y] = self.road(t)
    [dx,dy] = self.der_road(t)

    self.xr = x + offset*dy/np.sqrt(dx**2 + dy**2)
    self.yr = y - offset*dx/np.sqrt(dx**2 + dy**2)

    self.xl = x - offset*dy/np.sqrt(dx**2 + dy**2)
    self.yl = y + offset*dx/np.sqrt(dx**2 + dy**2)


  def noise_offset_road(self, mean=0, var=1):
    self.xrn = self.xr + np.random.normal(mean, var, self.size+1)
    self.yrn = self.yr + np.random.normal(mean, var, self.size+1)

    self.xln = self.xl + np.random.normal(mean, var, self.size+1)
    self.yln = self.yl + np.random.normal(mean, var, self.size+1)


  def get_road_points(self):
    pass


  def get_offset_road_points(self):
    return zip( np.around(self.xl,5).tolist() + np.around(self.xr,5).tolist() \
               ,np.around(self.yl,5).tolist() + np.around(self.yr,5).tolist()) 


  def get_noise_offset_road_points(self):
    pass


  def plot_offset_road(self):
    self._plot_road()
    self._plot_offset_road()
    self._plt.axis('equal')
    self._plt.show()


  def plot_noise_offset_road(self):
    self._plot_road()
    self._plot_noise_offset_road()
    self._plt.axis('equal')
    self._plt.show()


  def _load_figure_object(self):
    import matplotlib.pyplot as plt
    self._plt = plt
    self._figure_on = True


  def _plot_road(self):
    if not self._figure_on:
      self._load_figure_object()
    t = np.arange(self.t[0], self.t[-1]+0.01, 0.01)
    [x, y] = self.road(t)
    self._plt.plot(self.x, self.y, 'o')
    self._plt.plot(x ,y)


  def _plot_offset_road(self):
    if not self._figure_on:
      self._load_figure_object()
    self._plt.plot(self.xl, self.yl, 'o')
    self._plt.plot(self.xr, self.yr, 'o')


  def _plot_noise_offset_road(self):
    if not self._figure_on:
      self._load_figure_object()
    self._plt.plot(self.xln, self.yln, 'o')
    self._plt.plot(self.xrn, self.yrn, 'o')
