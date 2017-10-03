import numpy as np


def ellipse(a=18.0, b=10.0, bang=0.0, eang=2.0*np.pi, dis=50):
  th = np.arange(bang, eang+eang/dis, eang/dis)
  x = a*np.cos(th)
  y = b*np.sin(th)
  return list(zip(x,y))


def circle(r=10.0, bang=0.0, eang=2.0*np.pi, dis=50):
  return ellipse(r, r, bang, eang, dis)


def sine_wave(rev=1.0, fre=0.025, amp=1.0, dis=50):
  x = np.arange(0.0, rev/fre+rev/fre/dis, rev/fre/dis)
  y = amp*np.sin(2*np.pi*fre*x)
  return list(zip(x,y))
