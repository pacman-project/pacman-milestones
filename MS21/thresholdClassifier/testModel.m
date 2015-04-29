%function just makes exhaustive search over parametervector, need already a good specification otherwise it will take long
%X one row of X is single input vector, corresponding row of Y is training label
%training maximizes accuracy!

function confusionMatrix = testModel(param, X, Y, greaterSmaller)

if(size(X)!=size(Y))
  print("[Error] sizes of input and label differ!")
  exit 1;
endif

%size of parms either 1 or same as nr colums of X!!
%now for single!

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
    
    confusionMatrix = [TP FN; FP TN];

endfunction