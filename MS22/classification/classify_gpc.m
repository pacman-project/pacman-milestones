function worked=classify_gpc(dir_path)


subdirs = readdir (dir_path);
%%%loading data, skippin . and ..
%files{2}
size(subdirs);

raw_data=[];
selected_data=[];
labels=[];
alldirs=[];

%%create array training+novel and add for viewing

for(i=3:size(subdirs))

  j=i-2;
  current_dir=[dir_path "/" subdirs{i}];
  alldirs(j,:) = subdirs{i};
  [selected_data(j,:) raw_data(j,:) labels(j,:)]  = getFeaturesFromDir(current_dir);

endfor

selected_data;
labels;

%%format for gpc:

Y=labels.*2 -1; %converting to {+1,-1}
%X0=selected_data(:,2); %only haptics for now without prototype knowledge and alignment score:
X0=selected_data(:,2:end);
m = mean(X0,1);
sd =std(X0,0,1);
ons = ones(size(X0,1),1);
size(X0,1);
X = (X0-ons*m)./(ons*sd);

%%%Adding gpml package:
disp("Adding gmpl to current directory:");
addpath ("/home/alexander/external_libs/gpml-matlab-v3.4-2013-11-11/");
startup;

%%%4 times randomize with 75% percentage split: performance on test set with 15 entries
T_total=zeros(2,2);
T_total_novel=zeros(2,2);
T_total_training=zeros(2,2);

splitRate = 0.75;

X;
Y;
correct_ex = zeros(size(Y));
for  j=1:size(X,1) %%5 random splits

    %[x_train,y_train, x_test, y_test] = randSplitDataset(X,Y,splitRate);
    [x_train,y_train, x_test, y_test] = leaveOneOut(X,Y,j);

    meanfunc = @meanConst; hyp.mean = 0;
    covfunc = @covSEiso; 
    ell = 1.0; 
    sf = 1.0; 
    hyp.cov = log([ell sf]);
    likfunc = @likErf;

    hyp = minimize(hyp, @gp, -30, @infEP, meanfunc, covfunc, likfunc, x_train, y_train);
    %%prediction:
    [a b c d lp] = gp(hyp, @infEP, meanfunc, covfunc, likfunc, x_train, y_train, x_test, ones(length(x_test), 1));
    [a2 b2 c2 d2 lp2] = gp(hyp, @infEP, meanfunc, covfunc, likfunc, x_train, y_train, X, ones(length(X), 1));
    %%prediction to get correct:
    label_pred = (a2>0);
    label_orig = Y;
    correct_ex = correct_ex + (label_pred==label_orig);
    %%correctly classified:
    a;
    r=(a>0)';
    s=(y_test>0)'; %%labels back to (1,0)
    t=[r;s];
    
    %%Truthtable
    
    T=zeros(2,2);
    T_novel=zeros(2,2);
    T_training=zeros(2,2);
    
    for i=1:length(r)
    T(r(i)+1,s(i)+1) = T(r(i)+1,s(i)+1) + 1;
    
    if(j<20)
     T_training(r(i)+1,s(i)+1) = T_training(r(i)+1,s(i)+1) + 1;
     else
     T_novel(r(i)+1,s(i)+1) = T_novel(r(i)+1,s(i)+1) + 1;
    endif
    endfor
    
    T;
    T_total = T_total +T;
    T_total_training = T_total_training + T_training;
    T_total_novel = T_total_novel + T_novel;
    

    %%Performance:
    
    %disp("Precison")
    
    Precision = T(2,2)/(T(2,2) + T(2,1))
    
    
    
    %disp("Recall")
    
    Recall = T(2,2)/(T(2,2) + T(1,2))
    
    %disp("F1")
    
    F1 = 2*Precision*Recall/(Precision + Recall)
    
endfor
%filtering, e.g based on grasp_type or single template
disp("total precision")

T=T_total;
T
T_total_novel
T_total_training
Precision = T(2,2)/(T(2,2) + T(2,1))
   
    
    
    %disp("Recall")
    
    Recall = T(2,2)/(T(2,2) + T(1,2))
    
    %disp("F1")
    
    F1 = 2*Precision*Recall/(Precision + Recall)
    %disp("F1")
    %subdirs{3:end}
    correct_ex'

worked=true;

%%%%%%%which objects are not correctly classified based on haptics:




endfunction