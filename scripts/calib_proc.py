#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from argparse import ArgumentParser
from mpl_toolkits.axes_grid1 import make_axes_locatable

def pfun(v,w):
    # parametrization
    return [v,w]
pex = ['v','w']

def optimize_v2(src, dst, pfun, pex):
    v, w = src

    A = np.array(pfun(v,w)).T
    B = dst.T
    c, r = np.linalg.lstsq(A,B,rcond=None)[0:2]

    print 'v\' = c.v; c = ', c.T
    ci = np.linalg.inv(c.T)
    print 'v = ci.v; ci =', ci

    vex = 'v\' = ' + ' + '.join(['{}{}'.format(c_,v_) for c_, v_ in zip(c[:,0], pex)])
    wex = 'w\' = ' + ' + '.join(['{}{}'.format(c_,v_) for c_, v_ in zip(c[:,1], pex)])

    #print 'hmm', c
    #print 'hmm-r', r

    #vex = 'v\' = {}v + {}w + {}v.v + {}v.w + {}w.w'.format(*c[:,0])
    #wex = 'w\' = {}v + {}w + {}v.v + {}v.w + {}w.w'.format(*c[:,1])
    #vex = 'v\' = {}v + {}w'.format(*c[:,0])
    #wex = 'w\' = {}v + {}w'.format(*c[:,1])

    print ''
    print '=== Optimization Results : ==='
    print vex
    print wex
    print '=============================='
    print ''

    mfun = lambda v,w : c.T.dot(pfun(v,w))
    mifun = lambda v,w : ci.T.dot(pfun(v,w))
    return mfun, mifun

    #return lambda v,w : c.T.dot([v, w])
    #return lambda v,w : c.T.dot([v, w,1./v,1./w])

def delta_viz(
        mn_v, mx_v, mn_w, mx_w, v_res, w_res,
        src_v, src_w, dst_v, dst_w, f=None):
    nax = np.newaxis
    vs = np.arange(mn_v, mx_v, v_res)[::-1]
    ws = np.arange(mn_w, mx_w, w_res)

    def plot_delta(ax,viz,vs,ws,vmin,vmax,tick_skip=4):
        p = ax.imshow(viz,vmin=vmin,vmax=vmax)
        ax.set_xlabel('w')
        ax.set_ylabel('v')
        ax.set_xticks(range(len(ws))[::tick_skip])
        ax.set_xticklabels(np.around(ws,2)[::tick_skip], rotation=80)
        ax.grid(color='w')

        ax.set_yticks(range(len(vs))[::tick_skip])
        ax.set_yticklabels(np.around(vs,2)[::tick_skip])

        divider = make_axes_locatable(ax)
        cax = divider.append_axes('right', size='5%', pad=0.05)
        fig.colorbar(p, cax=cax)

    # apply model
    W, V = np.meshgrid(ws,vs)
    viz = f(V.ravel(),W.ravel())
    viz_dv_m = viz[0].reshape(V.shape) - V
    viz_dw_m = viz[1].reshape(W.shape) - W

    # apply data to field with l2 interpolation
    dv = np.reshape(vs, [-1,1]) - np.reshape(src_v, [1,-1]) # Gv,N
    dw = np.reshape(ws, [-1,1]) - np.reshape(src_w, [1,-1]) # Gw,N
    d = np.square(dv[:,nax]) + np.square(dw[nax,:]) # Gv,Gw,N
    weight = (1.0 / d)
    viz_dv_d = weight.dot(dst_v-src_v) / np.sum(weight, axis=-1) # or gw-cw
    viz_dw_d = weight.dot(dst_w-src_w) / np.sum(weight, axis=-1) # or gw-cw

    # get appropriate color map
    plt_vmin = min(viz_dv_m.min(), viz_dv_d.min())
    plt_vmax = max(viz_dv_m.max(), viz_dv_d.max())

    plt_wmin = min(viz_dw_m.min(), viz_dw_d.min())
    plt_wmax = max(viz_dw_m.max(), viz_dw_d.max())

    # model part
    fig, (ax0,ax1) = plt.subplots(2, 1, sharex=True)
    plot_delta(ax0,viz_dv_m,vs,ws,plt_vmin,plt_vmax)
    plot_delta(ax1,viz_dw_m,vs,ws,plt_wmin,plt_wmax)
    fig.suptitle('$\Delta v, \Delta \omega$ (model)', weight='bold', fontsize=14)

    # data part
    fig, (ax0,ax1) = plt.subplots(2, 1, sharex=True)
    plot_delta(ax0,viz_dv_d,vs,ws,plt_vmin,plt_vmax)
    plot_delta(ax1,viz_dw_d,vs,ws,plt_wmin,plt_wmax)
    fig.suptitle('$\Delta v, \Delta \omega$ (data)', weight='bold', fontsize=14)

def main():
    # arguments
    parser = ArgumentParser()
    parser.add_argument('-f', '--file', default='/tmp/data.npy', help='Calibration Data Path')

    parser.add_argument('-r', '--rel', dest='rel', action='store_true', help='Relative Discrepancy (default : absolute)')
    parser.add_argument('-i', '--inv', dest='inv', action='store_true', help='Invert optimization source to (gt_vel -> cmd_vel)')

    parser.set_defaults(rel=False, inv=False)

    opt = parser.parse_args()
    print('options : {}'.format(opt))

    # load file
    m = np.load(opt.file)
    gv, gw, cv, cw = m.T

    if opt.inv:
        mdir = 'gt->cmd' # == calculate cmd_vel from velocity
        src_v,src_w = gv,gw
        dst_v,dst_w = cv,cw
    else:
        mdir = 'cmd->gt' # == predict velocity from cmd_vel
        src_v,src_w = cv,cw
        dst_v,dst_w = gv,gw

    mfun, mifun = optimize_v2(
            np.stack([src_v,src_w], axis=0),
            np.stack([dst_v,dst_w], axis=0),
            pfun, pex)
    mv, mw = mfun(src_v,src_w)
    #mv, mw = mifun(dst_v, dst_w)

    print ' === Evaluation ==='
    print 'data  | error (v,w) :', np.square(gv - cv).sum() , np.square(gw - cw).sum()
    print 'model | error (v,w) : ', np.square(gv - mv).sum() , np.square(gw - mw).sum()
    print ' =================='

    # find limits
    mn_v, mx_v = np.min([gv,cv,mv]), np.max([gv,cv,mv])
    s_v = (mx_v - mn_v)
    avg_v = (mx_v + mn_v) / 2.0
    mn_w, mx_w = np.min([gw,cw,mw]), np.max([gw,cw,mw])
    s_w = (mx_w - mn_w)
    avg_w = (mx_w + mn_w) / 2.0
    s = max(s_v,s_w)
    lim_v = [avg_v - s/2., avg_v + s/2.]
    lim_w = [avg_w - s/2., avg_w + s/2.]

    x = src_w
    y = src_v
    u = (dst_w - src_w)
    v = (dst_v - src_v)
    u2 = (mw - src_w)
    v2 = (mv - src_v)


    if opt.rel:
        u /= (x + 0.01*np.sign(x))
        v /= (y + 0.01*np.sign(y))
        u2 /= (gw + 0.01*np.sign(w))
        v2 /= (gv + 0.01*np.sign(v))

    plt.figure()
    plt.axis('equal')
    plt.xlim(lim_w)
    plt.ylim(lim_v)
    plt.scatter(src_w,src_v,label='src')
    plt.quiver(x,y,u,v,color='k',label='{}(data)'.format(mdir), scale=1.0, units='xy')
    plt.quiver(x,y,u2,v2,color='r',label='{}(model)'.format(mdir), scale=1.0, units='xy' )
    plt.grid()
    plt.gca().set_axisbelow(True)

    plt.axvline(x=0.0, color='g', linestyle='-', label=None)
    plt.axhline(y=0.0, color='g', linestyle='-', label=None)


    plt.title('Command Velocity Discrepancy Field' + (' (Rel)' if opt.rel else '') )
    plt.xlabel('$\omega$')
    plt.ylabel('v')
    plt.legend()

    delta_viz(
        mn_v, mx_v, mn_w, mx_w, 0.05, 0.05,
        src_v, src_w, dst_v, dst_w, mfun)

    plt.show()

if __name__ == '__main__':
    main()
