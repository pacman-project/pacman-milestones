function result = threshold_classify()

%read in data
data_partpe = dlmread("scores_partpe.txt");
detection_idx_partpe=dlmread("detectionMat_partpe.txt");
labels = dlmread("labels.txt");

%%change nans to 100 for easy classification
ind = find(isnan(data_partpe));
data_partpe(ind)=100;

%%also change detection to score id of 100 for correct classification
indDet=find(isnan(detection_idx_partpe));
data_partpe(indDet)=100;

%%test training of part pe on labels:
thres_min_partpe = 6;
thres_max_partpe = 9;
greaterSmaller=false;
Y=labels';
X_partpe=data_partpe';

X_partpe;
Y;
[bestThres confMat] = trainModel(thres_min_partpe, thres_max_partpe, X_partpe, Y, greaterSmaller );

bestThres;
confMat;

accuracy=(confMat(1,1) + confMat(2,2))/(sum(sum(confMat)));
sensitivity=confMat(1,1)/(confMat(1,1) + confMat(1,2));
specificity=confMat(2,2)/(confMat(2,2) + confMat(2,1));

%%%leave one out cross validation:

size(X_partpe,1)

%total confMat and threshold List:

thresList=[];
confMatAll=zeros(2,2);

for i=1:size(X_partpe,1)

Xn = X_partpe;
Yn = Y;
%Get & Drop row
Xtest=X_partpe(i,:);
Ytest=Y(i,:);
Xn(i,:) = [];
Yn(i,:) = [];
%train
[bestThres confMat] = trainModel(thres_min_partpe, thres_max_partpe, Xn, Yn, greaterSmaller );
%test prediction
confResult = testModel(bestThres,Xtest,Ytest,greaterSmaller);
confMatAll = confMatAll + confResult;
thresList = [thresList bestThres];

endfor 

%%Results of classification!
confMatAll
thresList

TP=confMatAll(1,1)
TN=confMatAll(2,2)
FP=confMatAll(2,1)
FN=confMatAll(1,2)
confMat=confMatAll;
accuracy=(confMat(1,1) + confMat(2,2))/(sum(sum(confMat)))
sensitivity=confMat(1,1)/(confMat(1,1) + confMat(1,2))
specificity=confMat(2,2)/(confMat(2,2) + confMat(2,1))


endfunction


