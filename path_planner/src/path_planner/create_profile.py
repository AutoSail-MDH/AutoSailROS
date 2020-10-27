import numpy as np


class PointClass:
    def __init__(self, l_kx, l_ky, g_kx, g_ky, u):
        self.l_kx = l_kx
        self.l_ky = l_ky
        self.g_kx = g_kx
        self.g_ky = g_ky
        self.u = u


def create_profile(dim, pos_v):
    """
    Creates a list for all points in determined area around the boat with  local, global coordinates and the potential
    for that points.
    :param dim: he lenght of one side in the square matrix around the vessel for which potential is calculated.
    :param pos_v: The position of the vessel
    :return: A list for all points in determined area around the boat with  local, global coordinates and the potential
    for that points.
    """
    profile_ = []
    if (dim % 2) == 0:
        dim = dim + 1
    radius = (dim-1)/2
    l_range = np.linspace(-radius, radius, dim)
    list_len = dim * dim - 1

    for j in range(dim):
        for i in range(dim):
            p = PointClass(-radius+j, l_range[i], pos_v[0]-radius+j, pos_v[1]-radius+i, 0)
            profile_.append(p)
    return profile_, list_len


if __name__ == '__main__':
    diameter = 6
    position_v = np.array([9, 7])

    profile, list_len = create_profile(dim=diameter, pos_v=position_v)

    for k in range(list_len):
        print("p", profile[k].l_kx, profile[k].l_ky, profile[k].g_kx, profile[k].g_ky)

#    l_z_index = int(diameter * radius + radius)