#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from argparse import ArgumentParser

def main():
    # arguments
    parser = ArgumentParser()
    parser.add_argument('-f', '--file', default='/tmp/data.npy', help='Calibration Data Path')
    parser.add_argument('-r', '--rel', default='False', help='Relative / Absolute Discrepancy')
    parser.add_argument('-i', '--inv', default='False', help='Invert quiver source (default : cmd_vel -> gt_vel)')
    opt = parser.parse_args()

    # convert to bool (TODO : str2bool typing in argparse)
    opt.rel = (opt.rel.lower() in ['true', 'y', '1'])
    opt.inv = (opt.inv.lower() in ['true', 'y', '1'])

    # load file
    m = np.load(opt.file)
    gv, gw, cv, cw = m.T

    # find limits
    v = np.concatenate([gv,cv], axis=0)
    w = np.concatenate([gw,cw], axis=0)
    mn_v, mx_v = np.min(v), np.max(v)
    s_v = (mx_v - mn_v)
    avg_v = (mx_v + mn_v) / 2.0
    mn_w, mx_w = np.min(w), np.max(w)
    s_w = (mx_w - mn_w)
    avg_w = (mx_w + mn_w) / 2.0
    s = max(s_v,s_w)
    lim_v = [avg_v - s/2., avg_v + s/2.]
    lim_w = [avg_w - s/2., avg_w + s/2.]

    if opt.inv:
        x = gv
        y = gw
        u = (cv - gv)
        v = (cw - gw)
    else:
        x = cv
        y = cw
        u = (gv - cv)
        v = (gw - cw)

    if opt.rel:
        u /= (x + 0.01*np.sign(x))
        v /= (y + 0.01*np.sign(y))

    plt.quiver(x,y,u,v,color='k')
    plt.grid()
    plt.gca().set_axisbelow(True)

    plt.axvline(x=0.0, color='g', linestyle='-')
    plt.axhline(y=0.0, color='g', linestyle='-')

    plt.xlim(lim_v)
    plt.ylim(lim_w)


    plt.title('Command Velocity Discrepancy Field' + (' (Rel)' if opt.rel else '') )
    plt.xlabel('v')
    plt.ylabel('$\omega$')
    plt.show()

if __name__ == '__main__':
    main()
