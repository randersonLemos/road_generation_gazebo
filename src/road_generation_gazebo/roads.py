import utils
import numpy as np
from scipy import interpolate


class Road(object):
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
    if close:
        self.x = self.x + (self.x[0],)
        self.y = self.y + (self.y[0],)
    self.tck, self.t = interpolate.splprep([self.x, self.y], s=0, per=close, k=degree)
    self._figure_load = False


  def road(self, t):
    out = interpolate.splev(t, self.tck)
    return out


  def der_road(self, t, order=1):
    out = interpolate.splev(t, self.tck, der=order)
    return out


  def offset_road(self, offset, npts=100):
    t = np.arange(self.t[0], self.t[-1], self.t[-1]/npts)
    self.size = len(t)

    [x,y] = self.road(t)
    [dx,dy] = self.der_road(t)

    self.xr = x + offset*dy/np.sqrt(dx**2 + dy**2)
    self.yr = y - offset*dx/np.sqrt(dx**2 + dy**2)

    self.xl = x - offset*dy/np.sqrt(dx**2 + dy**2)
    self.yl = y + offset*dx/np.sqrt(dx**2 + dy**2)


  def noise_offset_road(self, var=1):
    self.xrn = self.xr + np.random.normal(0, var, self.size)
    self.yrn = self.yr + np.random.normal(0, var, self.size)

    self.xln = self.xl + np.random.normal(0, var, self.size)
    self.yln = self.yl + np.random.normal(0, var, self.size)


  def get_road_points(self):
    pass


  def get_offset_road_points(self):
    return zip( np.around(self.xl,5).tolist() + np.around(self.xr,5).tolist() \
               ,np.around(self.yl,5).tolist() + np.around(self.yr,5).tolist())


  def get_noise_offset_road_points(self):
    return zip( np.around(self.xln,5).tolist() + np.around(self.xrn,5).tolist() \
               ,np.around(self.yln,5).tolist() + np.around(self.yrn,5).tolist())


  def plot_show(self):
    self._plt.show()


  def plot_road(self):
    self._plot_road()
    self._plt.axis('equal')


  def plot_offset_road(self):
    self._plot_offset_road()
    self._plt.axis('equal')


  def plot_noise_offset_road(self):
    self._plot_noise_offset_road()
    self._plt.axis('equal')


  def _load_figure_object(self):
    import matplotlib.pyplot as plt
    self._plt = plt
    self._figure_load = True


  def _plot_road(self):
    if not self._figure_load:
      self._load_figure_object()
    t = np.arange(self.t[0], self.t[-1]+0.01, 0.01)
    [x, y] = self.road(t)
    self._plt.plot(self.x, self.y, 'o')
    self._plt.plot(x ,y, '--')


  def _plot_offset_road(self):
    if not self._figure_load:
      self._load_figure_object()
    self._plt.plot(self.xl, self.yl, '*')
    self._plt.plot(self.xr, self.yr, '*')


  def _plot_noise_offset_road(self):
    if not self._figure_load:
      self._load_figure_object()
    self._plt.plot(self.xln, self.yln, '^')
    self._plt.plot(self.xrn, self.yrn, '^')


  def make_clearance(self, width, npts=100):
    t = np.arange(self.t[0], self.t[-1], self.t[-1]/npts)

    [xs,ys] = self.road(t)

    self.xrn, self.yrn = self._clearance(xs,ys,self.xrn,self.yrn,width)
    self.xln, self.yln = self._clearance(xs,ys,self.xln,self.yln,width)

  def _distance_between_points(self, x, y, xp, yp):
      return np.sqrt((x-xp)**2 + (y-yp)**2)

  def _clearance(self, xs, ys, xps, yps, width):
    xxps = []
    yyps = []
    for xp,yp in zip(xps,yps):
        dists = []
        for x,y in zip(xs,ys):
            dists.append(self._distance_between_points(x,y,xp,yp))

        if min(dists) >= float(width)/2:
            xxps.append(xp)
            yyps.append(yp)

    return (np.array(xxps), np.array(yyps))

