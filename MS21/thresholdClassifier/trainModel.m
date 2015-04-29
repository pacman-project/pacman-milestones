%function just makes exhaustive search over parametervector, need already a good specification otherwise it will take long
%X one row of X is single input vector, corresponding row of Y is training label
%training maximizes accuracy!

function [bestParam, confusionMatrix] = trainModel(paramMin, paramMax, X, Y, greaterSmaller)

if(size(X)!=size(Y))
  print("[Error] sizes of input and label differ!")
  exit 1;
endif

%size of parms either 1 or same as nr colums of X!!
%now for single!
paramSplitSize=10;
deltaParam=abs(paramMax-paramMin)/paramSplitSize;
bestAccuracy=0;
for param=paramMin:deltaParam:paramMax

  thresholdMatrix=param*ones(size(X));
  prediction=[];
  if(greaterSmaller)
    prediction = (X>thresholdMatrix);
  else 
    prediction = (X<thresholdMatrix);
  endif
    
    TP=sum(sum( (-1*(prediction-1))==(Y-1) ));
    FP=sum(sum(prediction==(1+Y)));
    TN=sum(sum((-1*prediction)==(Y)));
    FN=sum(sum((1+prediction)==Y));
    
    accuracy = (TP+TN)/(TP+FP+TN+FN);
    if(accuracy>bestAccuracy)
      bestParam=param;
      confusionMatrix = [TP FN; FP TN];
      bestAccuracy=accuracy;    
    endif

endfor

endfunction