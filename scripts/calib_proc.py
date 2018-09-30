#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt

def main():
    m = np.load('/tmp/data.npy')
    m = m[:-4]
    gv, gw, cv, cw = m.T

    x = gv
    y = gw
    u = (cv - gv)
    v = (cw - gw)

    plt.quiver(x,y,u,v)
    plt.xlabel('v')
    plt.ylabel('w')
    plt.show()

if __name__ == '__main__':
    main()
