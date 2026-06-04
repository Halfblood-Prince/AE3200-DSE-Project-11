from math import pi
import numpy as np

t,d = None, None


class Bending:
    def __init__(self, t, d):
        self.t = t
        self.d = d
        self.inertia = pi*t*d**3/8
        self.Mx = None
        self.w = None
        self.rho = None

    def calculate_w(self, R, t):
        self.w = self.rho*pi*(2*R-t)*t

    def calculate_maximum_longitudinal_bending_moment(self, w, L, T):
        z_max = T/w - L
        self.Mx = T*L - (w*L**2)/2 + w*L*z_max - T*z_max - w*(z_max**2)/2

    def calculate_bending_stress(self, y):
        return self.Mx/self.inertia*y    

    

    




