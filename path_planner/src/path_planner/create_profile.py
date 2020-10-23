import numpy as np


class PointClass:
    def __init__(self, l_kx, l_ky, g_kx, g_ky, u):
        self.l_kx = l_kx
        self.l_ky = l_ky
        self.g_kx = g_kx
        self.g_ky = g_ky
        self.u = u


def create_profile(dim, pos_v):
    profile_ = []
    if (dim % 2) == 0:
        dim = dim + 1
    radius = (dim-1)/2
    l_range = np.linspace(-radius, radius, dim)

    for j in range(7):
        for i in range(dim):
            p = PointClass(-radius+j, l_range[i], pos_v[0]-radius+j, pos_v[1]-radius+i, 0)
            profile_.append(p)
    return profile_, dim


if __name__ == '__main__':
    diameter = 6
    position_v = np.array([9, 7])

    profile, diameter = create_profile(dim=diameter, pos_v=position_v)

    list_len = diameter * diameter - 1
    for k in range(list_len):
        print("p", profile[k].l_kx, profile[k].l_ky, profile[k].g_kx, profile[k].g_ky)

#    l_z_index = int(diameter * radius + radius)