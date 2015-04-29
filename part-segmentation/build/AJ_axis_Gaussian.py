"""
AJ_axis_Gaussian
Copyright: Antonio Rodriguez-Sanchez
           Intelligent and Interactive Systems
           University of Innsbruck
Free for academic purposes

Call AJ_axis_Gausian.axis(filename, display)
This program computes the axis of a file (filename) whose coordinates are in csv format.
File must contain coordinates in the first 3 columns and normals in the next 3 columns
It follows the method from Sun and Sherrah (PAMI, 1997)
Use show=1 if you want to display the axis
"""

from numpy import mean, concatenate, median, tile, dot, diag, power, eye, vstack
from numpy import ones, zeros, where, outer, sqrt, array, abs, copy, trace, matrix
from numpy import sum as np_sum
from numpy import max as np_max
from numpy import linalg, reshape, histogram2d, random, logical_and, meshgrid, exp
from numpy import genfromtxt, ndenumerate, size, arange, ndindex, newaxis, sinh, log
from math import pi, cos, sin
from mpl_toolkits.mplot3d import Axes3D
from collections import namedtuple
# from pylab import show, figure
from matplotlib.pyplot import hist2d, close, grid, show, figure, savefig, axis
import mmr_normalization_new
from itertools import product as it_prod
from sys import exit
from os import system

def axis(fname, disp):
          
        xyz = genfromtxt(fname,delimiter=',')
        #print("after getting data")       
        (u,i,umax) = mmr_normalization_new.mmr_geometricmedian(xyz[:,0:3])
        # d = density_von_mises_fisher(xyz[:,3:6],200)
        # print(np_max(d))
        # J = inertia_matrix(xyz[d>0.95*np_max(d),0:3])

        bins = 3

        J = inertia_matrix(xyz[:,0:3])
        (eig_val, eig_vec) = linalg.eig(J)
        ind_max = eig_val.argmax(axis=0)
        
        p_axesx = eig_vec[:,0]
        p_axesy = eig_vec[:,1]
        p_axesz = eig_vec[:,2]
 #       print("eigvec calculated")
        
        for j in range(1,3):
            bins = j*bins
  #          print(xyz[0:1,3])
   #         print(xyz[0:1,4])
    #        print(xyz[0:1,5])
            (hist,xyzs,ax) = hist3d_scatter(abs(xyz[:,3]),xyz[:,4],xyz[:,5],bins,True)
     #       print("scatter done")
            if(ind_max==0):
                (p_axesx,corra1) = reflect_symmetry(p_axesx,0,hist,xyzs)
            elif(ind_max==1):
                (p_axesy,corra2) = reflect_symmetry(p_axesy,1,hist,xyzs)
            elif(ind_max==2):
                (p_axesz,corra3) = reflect_symmetry(p_axesz,2,hist,xyzs)

      #  print("to get eigvec array")
        v1 = array(vstack([u-p_axesx,u,u+p_axesx.T]))
        v2 = array(vstack([u-p_axesy,u,u+p_axesy.T]))
        v3 = array(vstack([u-p_axesz,u,u+p_axesz.T]))

        if disp ==1:
          show(xyz,v1,v2,v3)
        #print(v1,v2,v3)
        return(v1,v2,v3,ind_max)

def reflect_symmetry(p_axes,ne,hist,xyzs):

    prev_corr = 0
    length = hist.shape[0]
    inc = xyzs[0][1]-xyzs[0][0], xyzs[1][1]-xyzs[1][0], xyzs[2][1]-xyzs[2][0]
    mm = array(xyzs).max(1), array(xyzs).min(1)
    if(ne==0):
        it = it_prod(range(0,1,1),range(-1,2,1),range(-1,2,1))
    elif(ne==1):
        it = it_prod(range(-1,2,1),range(0,1,1),range(-1,2,1))        
    elif(ne==2):
        it = it_prod(range(-1,2,1),range(-1,2,1),range(0,1,1))
    corr = zeros(27)
    k = 0
    for j in it:
        neigh = array(j)*array(inc)
        p_axes2 = p_axes+neigh.T
        mir = eye(3,3)-2*(outer(p_axes2,p_axes2))
        corr[k] = 0
        for i in range(length):
            v = hist[i,0:3]+array(inc)/2
            h = hist[i,3]
            mir_v = dot(mir,v)
            #if(all(logical_and(mir_v>=mm[1],mir_v<=mm[0]))):
            a = logical_and(mir_v>hist[0:length,0:3],mir_v<=(hist[0:length,0:3]+inc))
            corr[k] = corr[k]+abs(dot(v,p_axes2))      
#        print(corr[k])          
        if(corr[k]>prev_corr):
       #     print("refine corre")
            prev_corr = corr[k]
            f_axes = p_axes2
        k = k+1
        
    row_sums = corr.sum(axis=0)
   # print("row_sums")
    #print(row_sums[newaxis])
    #norm_corr = corr / row_sums[newaxis]
    # print(row_sums)
    return(f_axes,row_sums)

def density_von_mises_fisher(X,kappa):
  """
  Returns the unnormalized density estimation of a point cloud from each point of   the cloud based on the von Mises-Fisher distribution
  Input:
        X   array containing the vectors corresponding to the points in its row
        kappa   scalar variance of the spherical distribution,
                if it is greater then the distribution is more concentrated
                
  Output:
        densityX  vector contains the value of the density estimation
                  in each point.
  """

  (m,n)=X.shape

  xnorm=sqrt(np_sum(X**2,axis=1))
  xnorm=xnorm+(xnorm==0)
  X=X/outer(xnorm,ones(n))

  K=dot(X,X.T)
  kappaK=kappa*K
  ekappaK=exp(kappaK)
  C_3kappa=kappa/(4*pi*sinh(kappa))
  cekappaK=C_3kappa*ekappaK

  densityX=cekappaK.sum(0)/m
  
  return(densityX)
## #############################################

def show(xyz, axis1, axis2, axis3):

    fig = figure()
    ax = Axes3D(fig)
    ax.scatter(xyz[:,0],xyz[:,1],xyz[:,2])
    ax.plot(axis1[:,0],axis1[:,1],axis1[:,2],linewidth=4,color='g')
    ax.plot(axis2[:,0],axis2[:,1],axis2[:,2],linewidth=4,color='r')
    ax.plot(axis3[:,0],axis3[:,1],axis3[:,2],linewidth=4,color='b')
    ax.set_xticks(arange(-1,1,0.2))
    ax.set_yticks(arange(-1,1,0.2))
    ax.set_zticks(arange(-1,1,0.2))
    grid()
    fig.show()

def show_plane(xyz, normal, point):
    
    d = -sum(point*normal)# dot product

    print(normal)
    xx, yy = meshgrid(arange(-1,1,.01), arange(-1,1,.01))
    z = (-normal[0]*xx - normal[1]*yy - d)/normal[2]


    fig = figure()
    ax = Axes3D(fig)
    ax.scatter(xyz[:,0],xyz[:,1],xyz[:,2])   # plot the surface
    plt3d = fig.gca(projection='3d')
    plt3d.plot_surface(xx,yy,z)
    ax.set_xticks(arange(-1,1,0.2))
    ax.set_yticks(arange(-1,1,0.2))
    ax.set_zticks(arange(-1,1,0.2))
    ax.auto_scale_xyz([-1,1],[-1,1],[-1,1])
    fig.show()

    
def inertia_matrix(xyz):

    m = xyz.mean(axis=0)
    A = array(xyz-tile(m,(size(xyz,axis=0),1)))
    C = dot(A.T,A)
    J = dot(trace(C),eye(3,3))-C
    J = J/linalg.norm(J)
    return(J)

def normalize_shape(xyz):

    xyz[:,0] *= 1/(xyz[:,0].max()-xyz[:,0].min())
    xyz[:,1] *= 1/(xyz[:,1].max()-xyz[:,1].min())
    xyz[:,2] *= 1/(xyz[:,2].max()-xyz[:,2].min())
    return xyz

def hist3d_scatter( x_data, y_data, z_data, bins=10, bubbles=False, plot=False):
 
    #print("in hist3d scatter")
    #print("before hist2d 1")
    #print(x_data)
    #print(y_data)
    ax1 = hist2d( x_data, y_data, bins=bins)
   # print("after hist2d 1")
    close(ax1[-1].get_figure().number)
    ax2 = hist2d( x_data, z_data, bins=bins)
    #print("after hist2d 2")
    close(ax2[-1].get_figure().number)
    ax3 = hist2d( y_data, z_data, bins=bins)
    #print("after hist2d 3")
    close(ax3[-1].get_figure().number)
    #print("after hist2d all")
    xs, ys, zs = ax1[2], ax1[2], ax3[2]
    dx, dy, dz = xs[1]-xs[0],  ys[1]-ys[0], zs[1]-zs[0]
    
    def rdn():
        return (1-(-1))*random.random() + -1
    smart = zeros((bins,bins,bins),dtype=int)
    for (i1,j1),v1 in ndenumerate(ax1[0]):
        if v1==0: continue
        for k2,v2 in enumerate(ax2[0][i1]):
            v3 = ax3[0][k2][j1]
            if v1==0 or v2==0 or v3==0: continue
            num = min(v1,v2,v3)
            smart[i1,j1,k2] += num
            v1 -= num
            v2 -= num
            v3 -= num
    points = []
    points_plt = []
    for (i,j,k),v in ndenumerate(smart):
        if bubbles:
            points.append( (xs[i],ys[j],zs[k],v) )
        ## else:
        for m in range(int(v)):
            x = xs[i] + rdn()*dx/6
            y = ys[j] + rdn()*dy/6
            z = zs[k] + rdn()*dz/6
            points_plt.append( (x,y,z) )
    points_plt = array(points_plt)
    points = array(points)
    if plot:
        fig = figure()
        sub = fig.add_subplot(111, projection='3d')
        ## if bubbles:
        ##     sub.scatter(points[:,0], points[:,1], points[:,2],
        ##             color='black', marker='o',s=128*points[:,3])
        ## else:
        sub.scatter(points_plt[:,0], points_plt[:,1], points_plt[:,2],
                    color='black', marker='x',s=64)
        sub.axes.set_xticks( xs )
        sub.axes.set_yticks( ys )
        sub.axes.set_zticks( zs )
        fig.show()
        
    return(points,[xs,ys,zs],[ax1,ax2,ax3])

def sph2cart(angle):
    # angle  - [theta,phi] colatitude and longitude
    theta,phi = angle
    z = cos(theta)
    x = sin(theta)*cos(phi)
    y = sin(theta)*sin(phi)
    return x,y,z

# testing other things

def axis_cyl():

    xyz = genfromtxt('../Princeton4/m1813.csv', delimiter=',')
    (u,i,umax)=mmr_normalization_new.mmr_geometricmedian(xyz[:,0:3])
    std = xyz.std(axis=0)
    std2 = std[0:3]
    val = 2*std.min(axis=0)
    r = 10
    print(val)
    cyl = namedtuple('cyl',['minx','maxx','miny','maxy','minz','maxz'])
    cyl_env = cyl(min(xyz[:,1])-val,max(xyz[:,1])+val, \
            min(xyz[:,2])-val,max(xyz[:,2])+val, \
            min(xyz[:,3])-val,max(xyz[:,3])+val)
    
    min_sd = float("inf")
    for th in range(0,360,5):
        for ph in range(0,360,5):
            theta = th*pi/180
            phi = ph*pi/180
            w = sph2cart([theta,phi])
            xi = xyz[:,0:3]-u
            D =  (xi-outer(dot(xi,w),w))**2
            diff = (r-sqrt(D.sum(axis=1)))**2
            sum_diff = diff.sum(axis=0)
            if sum_diff<min_sd:
                min_sd = sum_diff
                w_fin = w
    print(w_fin)
    cyl_points = array(cyl_env)
    cyl_points2 = cyl_points.reshape(2,3)
    v = array([u-w_fin,u,u+w_fin])
    fig = figure()
    ax = Axes3D(fig)
    ax.scatter(xyz[:,0],xyz[:,1],xyz[:,2])
    ax.scatter(cyl_points2[:,0],cyl_points2[:,1],cyl_points2[:,1],color='r')
    ax.plot(v[:,0],v[:,1],v[:,2],linewidth=4,color='r')
    show()
    return(u,w_fin)


