from numpy import mean, repeat, shape, isclose, around, arange, where, genfromtxt, reshape
from numpy import zeros, outer, sqrt, array, abs, argsort, log2, tile, kron, ones, repeat
from numpy import sum as np_sum
from numpy import max as np_max
from numpy import min as np_min
from matplotlib.pyplot import grid, show, figure, savefig, axis
from itertools import product as it_prod
from sys import exit
from os import system
from mpl_toolkits.mplot3d import Axes3D
from math import log

def entropy_PC():

    # xyz = genfromtxt('../Princeton5/m1598.csv', delimiter=',')
    # xyz = genfromtxt('../Princeton5/m807.csv', delimiter=',')
    # xyz = genfromtxt('../Princeton5/m1167.csv', delimiter=',')
    # for i in range(1,2):
    # for i in range(649,686):
    for i in range(783,784):
        # fn = str(random.randint(0,1814))
        fn = str(i)
        # fn='783'
        # fn = '506'
        # fn = '1450'
        neigh = 20
        print(fn)
        fname = '../../Princeton5/m'+fn+'.csv'
        xyz = genfromtxt(fname, delimiter=',')
        # xyz[:,0:3] = normalize_shape(xyz[:,0:3])

        length = xyz.shape[0]
        # flags = ones((length))
        H = zeros(length)
        cnt = 0

        while cnt<length: # and np_sum(flags)>0:
            p1 = xyz[:,0:3]
            p2 = xyz[cnt,0:3]
            ed = sqrt(np_sum((p1-p2)**2,axis=1))
            ind = argsort(ed, kind='mergesort')
            # print(ind, xyz[cnt,0:3])
            diffN = abs(kron(xyz[cnt,3:6],ones((neigh,1)))-xyz[ind[0:neigh],3:6])
            # diffN = xyz[ind[0:neigh],3:6]+1
            diffN = around(diffN,decimals=1) 
            xi = arange(0, 2, 0.1)
            yi = arange(0, 2, 0.1)
            zi = arange(0, 2, 0.1)
            rng = list(it_prod(xi, yi, zi))
            # print(len(rng))
            H[cnt] = entropy(diffN,rng)
            #flags[ind] = 0;
            print(cnt, H[cnt])
            # ind_next = ind[ind>=cnt]
            cnt += 1
            # cnt = np_min(ind_next)+1
            # print(diffN)
        # savefig('./Results/m'+fn+'_sym')
 
    show_entropy(xyz, H)
    return cnt


def entropy(data, rb):
#    if not data:
#        return 0
#    iterator = range(rb)
    entropy = 0
    # print(data)
    # print(data.shape)
    data_rep = repeat(data,len(rb),axis=0)
    # data_rep = tile(array(data),(len(rb),1))
    # rb_rep = repeat(array(rb),len(data),axis=0)
    rb_rep = tile(array(rb),(len(data),1))
    #print(array(data_rep).shape, array(rb_rep).shape)
    # print(data_rep, rb_rep)
    
    p_xaux = isclose(data_rep,rb_rep,0,0.1)
    #print(p_xaux.shape)
    p_xaux = p_xaux.all(axis=1)
    #print(p_xaux.shape)
    # print(rb_rep[where(p_xaux>0)])
    p_x = reshape(p_xaux,(len(data),-1))
    rb_aux = array(rb) # reshape(rb_rep,(-1,3))
    #print(p_x.shape)
    p_x = np_sum(p_x,axis=0)/len(data)
    #print(rb_aux[where(p_x>0),:])
    #print(p_x[where(p_x>0)])
    entropy = - np_sum(p_x[where(p_x>0)]*log2(p_x[where(p_x>0)]))
    #print(entropy)
    
    ## cnt = 0
    ## for x in rb:
    ##     # print(data,array(x),len(data), entropy)
    ##     # p_x = float(list(data).count(x)/len(data))
    ##     # x_rep = kron(x,ones((len(data),1)))
    ##     x_rep = tile(array(x),(len(data),1))
    ##     # print(x_rep.shape)
    ##     sub = zeros((len(data),3))+0.001
    ##     # print(isclose(data,x_rep),0,0.01)
    ##     p_xaux = isclose(data,x_rep,0,0.1)
    ##     p_x = float(p_xaux.all(axis=1).sum())/len(data)
    ##     if p_x > 0:
    ##         entropy += - p_x*log(p_x, 2)
    ##         print(x,p_x,entropy)
    ##         cnt = cnt+1
    ## print(cnt)
    return(entropy)

def show_entropy(xyz, H):

    H_color = (H-np_min(H))/(np_max(H)-np_min(H))
    fig = figure()
    ax = Axes3D(fig)
    xyz_b = xyz[where(H_color<=0.25)[0]]
    xyz_r = xyz[where(H_color>0.75)[0]]
    print(np_min(H),np_max(H),H[1],H_color[1], H_color, xyz_b.shape, xyz_r.shape)
    # xyz_g = xyz[where((H_color>0.25) and (H_color<=0.75))[0]]

    # ax.scatter(xyz[:,0],xyz[:,1],xyz[:,2], c=H_color)
    ax.scatter(xyz_b[:,0],xyz_b[:,1],xyz_b[:,2], color='b')
    # ax.scatter(xyz_g[:,0],xyz_g[:,1],xyz_g[:,2], color='g')
    ax.scatter(xyz_r[:,0],xyz_r[:,1],xyz_r[:,2], color='r')
    ax.set_xticks(arange(-1,1,0.2))
    ax.set_yticks(arange(-1,1,0.2))
    ax.set_zticks(arange(-1,1,0.2))
    grid()
    fig.show()





## def range_bytes (): return range(256)
## def range_printable(): return (ord(c) for c in string.printable)
## def H(data, iterator=range_bytes):
##     if not data:
##         return 0
##     entropy = 0
##     for x in iterator():
##         p_x = float(data.count(chr(x)))/len(data)
##         print(chr(x),data.count(chr(x)),len(data),p_x, entropy)
##         if p_x > 0:
##             entropy += - p_x*log(p_x, 2)
##     return entropy

## def main ():
##     for row in fileinput.input():
##         string = row.rstrip('\n')
##         print ("%s: %f" % (string, H(string, range_printable)))

## for str in ['gargleblaster', 'tripleee', 'magnus', 'lkjasdlk',
##                'aaaaaaaa', 'sadfasdfasdf', '7&wS/p(']:
##     print ("%s: %f" % (str, H(str, range_printable)))

if __name__=="__main__":

    entropy_PC()
    

