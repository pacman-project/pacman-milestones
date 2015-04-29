function [Atrain, ytrain, Atest ytest ] = leaveOneOut(A,y, i)

%  selcts the i_th item as testData

[N D] = size(A);
[N2 D2] = size(y);

if N != N2 
 error ("sizes of input and output dimension don't match!");
endif

if i > N 
 error ("sizes of ith item is too large match!");
endif

data = [A y]; %Appending A and y
IDX =  [(1:i-1) (i+1:N)];

trainData = data(IDX, :);
testData = data(i, :);

%split again
Atrain=trainData(:,1:D);
ytrain=trainData(:,D+1:D+D2);

Atest=testData(:,1:D);
ytest=testData(:,D+1:D+D2);


end
