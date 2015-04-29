######################
## Version 0.1 #######
######################
from numpy import mean, concatenate, median, tile, dot, diag
from numpy import ones, zeros, where, outer, sqrt, array, abs, copy
from numpy import sum as np_sum
from numpy import max as np_max
from numpy import linalg
## ###########################################################3
def mmr_normalization(ilocal,iscale,XTrain,XTest,ipar):
## function to normalize the input and the output data
## !!!! the localization happens before normalization if both given !!! 
## input
##      ilocal centralization   
##                  =-1 no localization
##                  =0 mean
##                  =1 median
##                  =2 geometric median
##                  =3 shift by ipar
##                  =4 smallest enclosing ball
##      icenter
##                  =-1 no scaling
##                  =0 scale item wise by L2 norm
##                  =1 scale item wise by L1 norm
##                  =2 scale item wise by L_infty norm
##                  =3 scale items by stereographic projection relative to zero
##                  =4 scale variables by STD(standard deviation)
##                  =5 scale variables by MAD(median absolute deviation)
##                  =6 scale variables by absolute deviation
##                  =7 scale all variables by average STD 
##                  =8 scale all variables by maximum STD 
##                  =9 scale all variables by median MAD 
##                  =10 scale item wise by Minkowski norm, power given by ipar
##                  =11 \sum_i||u-x_i||/m where u=0
##                  =12 scale all variables by overall max
##                  =13 Mahalonobis scaling  
##      XTrain       Data matrix which will be normalized. It assumed the
##                   rows are the sample vetors and the columns are variables 
##      XTest        Data matrix which will be normalized. It assumed the
##                   rows are the sample vetors and the columns are
##                   variables.
##                   It herites the center and the scale in the variable wise ca##                   se from the XTrain,
##                   otherwise it is normalized independently  
##      ipar         additional parameter   
##  output
##      XTrain       Data matrix which is the result of the normalization
##                   of input XTrain. It assumed the rows are the sample
##                   vetors and the columns are variables  
##      XTest        Data matrix which is the result of the normalization
##                   of input XTest. It assumed the rows are the sample
##                   vetors and the columns are variables.
##      opar         the radius in case of ixnorm=2.  
##  
  opar=0;
  (mtrain,n)=XTrain.shape
  if len(XTest.shape)>=2:
    mtest=XTest.shape[0]
  elif len(XTest.shape)==1:
    mtest=XTest.shape[0]
    XTest=XTest.reshape((1,mtest))
  else:
    mtest=0
    XTest=array([])

  if ilocal==-1:
    pass
  elif ilocal==0:   ##  mean
    xcenter=mean(XTrain,axis=0)
  elif ilocal==1:   ##  median
    xcenter=median(XTrain,axis=0)
  elif ilocal==2:    ##  geometric median
    xcenter=mmr_geometricmedian(XTrain)[0]
  elif ilocal==3:    ##  shift by ipar
    xcenter=ipar
  elif ilocal==4:   ##  smallest comprising ball
    xalpha=mmr_outerball(0,XTrain)
    xcenter=dot(XTrain.T,xalpha)

  if ilocal in (0,1,2,3,4):
    XTrain=XTrain-tile(xcenter,(mtrain,1))
    if mtest>0:
      XTest=XTest-tile(xcenter,(mtest,1))

## itemwise normalizations
  if iscale==-1:
    pass
  elif iscale==0:     ## scale items by L2 norm
    xscale_tra=sqrt(np_sum(XTrain**2,axis=1))
    if mtest>0:
      xscale_tes=sqrt(np_sum(XTest**2,axis=1))
  elif iscale==1:     ## scale items by L1 norm
    xscale_tra=np_sum(abs(XTrain),axis=1)
    if mtest>0:
      xscale_tes=np_sum(abs(XTest),axis=1)
  elif iscale==2:     ## scale items by L_infty norm
    xscale_tra=np_max(abs(XTrain),axis=1)
    if mtest>0:
      xscale_tes=np_max(abs(XTest),axis=1)
  elif iscale==10:     ## scale items by Minowski with ipar
    xscale_tra=np_sum(abs(XTrain)**ipar,axis=1)**(1/ipar)
    if mtest>0:
      xscale_tes=np_sum(abs(XTest)**ipar,axis=1)**(1/ipar)

  if iscale in (0,1,2,10):    
    xscale_tra=xscale_tra+(xscale_tra==0)
    XTrain=XTrain/tile(xscale_tra.reshape(mtrain,1),(1,n))
    if mtest>0:
      xscale_tes=xscale_tes+(xscale_tes==0)
      XTest=XTest/tile(xscale_tes.reshape(mtest,1),(1,n))
          
  if iscale==3:   ## scale items by stereographic projection relative to zero
    xnorm2=np_sum(XTrain**2,axis=1)
    R=ipar
    xhom=ones(mtrain)/(xnorm2+R**2)
    xhom2=xnorm2-R**2
    XTrain=concatenate((2*R**2*XTrain*outer(xhom,ones(n)),R*xhom2*xhom), \
                       axis=1)
    if mtest>0:
      xnorm2=np_sum(XTest**2,axis=1)
      xhom=ones(mtest)/(xnorm2+R**2)
      xhom2=xnorm2-R**2
      XTest=concatenate((2*R**2*XTest*outer(xhom,ones(n)),R*xhom2*xhom), \
                        axis=1)

## variable wise normalization relative to zero
## test has to use of the training scale 

  if iscale==-1:
    pass
  elif iscale==4:     ## scale vars by std to zeros center
##    xscale=std(XTrain,axis=0)
    xscale=sqrt(mean(XTrain**2,axis=0)) 
  elif iscale==5:     ## scale vars by mad
    xscale=median(abs(XTrain),axis=0)
  elif iscale==6:     ## scale vars by absolut deviation
    xscale=mean(abs(XTrain),axis=0)

  if iscale in (4,5,6):
    xscale=xscale+(xscale==0)
    XTrain=XTrain/tile(xscale,(mtrain,1))
    if mtest>0:
      XTest=XTest/tile(xscale,(mtest,1))

  if iscale==-1:
    pass
  if iscale==7:     ## scale vars by average std to zero center
##    xscale=mean(std(XTrain,axis=0))
    xscale=mean(sqrt(mean(XTrain**2,axis=0)))
  elif iscale==8:     ## scale vars by max std to zero center
##    xscale=np_max(std(XTrain,axis=0))
    xscale=np_max(sqrt(mean(XTrain**2,axis=0)))
  elif iscale==9:     ## scale vars by median mad
    xscale=median(median(abs(XTrain),axis=0))
  elif iscale==11:    ## \sum_i||u-x_i||/m where u=0
    xscale=mean(sqrt(np_sum(XTrain**2,axis=1)))
  elif iscale==12:    ## \sum_i||u-x_i||/m where u=0
    xscale=XTrain.max()

##  print(xscale)
  if iscale in (7,8,9,11,12):
    xscale=xscale+(xscale==0)
    XTrain=XTrain/xscale
    if mtest>0:
      XTest=XTest/xscale

  if iscale==13:     ## scale by Mahalonobis
    xsigma=dot(XTrain.T,XTrain) ## covariance
    [w,v]=linalg.eigh(xsigma)
    iw=where(w<=10**(-10))[0]
    w[iw]=0.0
    iw=where(w>0.0)[0]
    w_sqinv=zeros(XTrain.shape[1])
    w_sqinv[iw]=1/sqrt(w[iw])
    XTrain=dot(XTrain,v)*outer(ones(mtrain),w_sqinv)
    if mtest>0:
      XTest=dot(XTest,v)*outer(ones(mtest),w_sqinv)
    
  return(XTrain,XTest,opar)
    
## ******************************************************
def mmr_outerball(iker,xdata):
## solves the minimum enclosing ball problem via one-class maximum margin dual   
  m=xdata.shape[0]
  if iker==1:    ## kernel input
    K=xdata
  else:
    K=dot(xdata,xdata.T)
  kdiag=diag(K)
  niter=1000
  l1eps=0.02/m
  alpha=ones(m)/m
##  alpha=rand(m,1)
##  alpha=alpha/np_sum(abs(alpha))
  for iiter in range(niter):
## compute the gradient
##    fgrad=2*K*alpha-kdiag
    fgrad=dot(K,alpha)-kdiag/2
## solve subproblem:   min_u fgrad'*u, s.t. 1'*u=1, u>=0
    u=zeros(m)
    vm=min(fgrad)
    imm=where(fgrad==vm)[0]
    limm=len(imm)
    if limm==0:
      limm=1
##      u(imm(randi(limm,1)))=1
    u[imm]=float(1)/limm
    xdelta=u-alpha
    xdenom=dot(dot(xdelta,K),xdelta)
    if xdenom==0:
      break
    tau=-dot(fgrad,xdelta)/xdenom
    if tau<=0:
      tau=0
      break
    if tau>1:
      tau=1
##    print(iiter,tau)
    alphanew=tau*u+(1-tau)*alpha
##    f=(alphanew'*K*alphanew-kdiag'*alphanew)/2
    alphadiff=tau*xdelta
    xerr=np_sum(abs(alphadiff))
##    disp([iiter,tau,xerr,f])
    if xerr/m<l1eps: 
      alpha=alphanew
      break
    alpha=alphanew
  
##  xcenter=dot(xdata.T,alpha)

  return(alpha)
## ####################################################
""" Weszfeld's algorithm:
Weiszfeld, E. (1937). "Sur le pour lequel la somme des distances de n points donnes est minimum" Tohou Math. Journal 43:355-386
"""
def mmr_geometricmedian(X):
  (m,n)=X.shape
  u=mean(X,axis=0)
  niter=1000
  xeps=sqrt(np_sum(u**2))/1000
  xerr=2*xeps
  for i in range(niter):
    d2u=sqrt(np_sum((X-tile(u,(m,1)))**2,axis=1))
    inul=where(d2u<xeps)[0]
    d2u[inul]=xeps
    unext=np_sum(X/tile(d2u.reshape((m,1)),(1,n)),axis=0)/np_sum(ones(m)/d2u)
    if np_max(unext-u)<xerr:
      break
    u=copy(unext)
  return(unext,i,np_max(unext-u))

## ####################################################
""" Weszfeld's algorithm for kernel representation:
Weiszfeld, E. (1937). "Sur le pour lequel la somme des distances de n points donnes est minimum" Tohou Math. Journal 43:355-386
"""
def mmr_geometricmedian_ker(K):
  m=K.shape[0]
  Ka=mean(K,axis=1)
  niter=1000
  xeps=sqrt(np_sum(Ka**2))/1000
  xerr=2*xeps
  Q=linalg.qr(K)[0]
  Ra=dot(Q.T,Ka)
  aKa=dot(Ra,Ra)
  for i in range(niter):
    d2u=sqrt((zeros(m)+aKa)+diag(K)-2*Ka)
    inul=where(d2u<xeps)[0]
    d2u[inul]=xeps
    Kanext=np_sum(K/tile(d2u.reshape((m,1)),(1,m)),axis=0) \
           /np_sum(ones(m)/d2u)

    du=np_sum((K-tile(Ka,(m,1))).T/tile(d2u,(m,1)),axis=1)
    print(sqrt(np_sum(du**2)))
    if np_max(Kanext-Ka)<xerr:
      Ka=copy(Kanext)
      Ra=dot(Q.T,Ka)
      aKa=dot(Ra,Ra)
      break
    Ka=copy(Kanext)
    Ra=dot(Q.T,Ka)
    aKa=dot(Ra,Ra)
    
  return(Ka,aKa)


    
    
