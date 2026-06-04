from math import pi
import numpy as np

t,d = None, None


class Bending:
    def __init__(self, t, d):
        self.t = t
        self.d = d
        self.inertia = pi*t*d**3/8

    def calculate_bending_stress(self, Mx, My, x, y):
        return (Mx*y + My*x)/self.inertia
    
    
    




