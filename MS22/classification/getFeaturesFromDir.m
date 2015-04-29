%%DATA:

%%RAW = A_selected = (score_mes(1x2=graspscore prototype_id), kinesthetic_mes(1x7), tactile_f1(13x4->1x78), tactile_f2(13x6->1x78), tactile_f3(13x4->1x78))

%%A_selected= (score_mes(1x2), kinesthetic_compared_to_closed_pos(1), tactile_f1(m0(1x2), I0, Cov_I(1x2), tactile_f2(m0(1x2), I0, Cov_I(1x2)), tactile_f3(m0(1x2), I0, Cov_I(1x2)) )

%%number of pixels also interesting feature? 2d median?

function [A_selected, A_raw, Label] = getFeaturesFromDir(path_dir)

file_list = {"/kinesthetic_mes.txt", 
	     "/score_mes.txt",
	     "/success.txt",
	     "/tactile_mes.txt"};

data_kin=dlmread([path_dir file_list{1}]);
data_score=dlmread([path_dir file_list{2}]);
data_success=dlmread([path_dir file_list{3}]);
data_tactile=dlmread([path_dir file_list{4}]);

%%Finger 2 is thumb
tactile_f1=data_tactile(2,1:78);
tactile_f2=data_tactile(4,1:78); %ist thumb
tactile_f3=data_tactile(6,1:78);

%for computing moments:
img1=reshape(tactile_f1,13,6);
img2=reshape(tactile_f2,13,6);
img3=reshape(tactile_f3,13,6);

M1=getMoments(img1);
M2=getMoments(img2);
M3=getMoments(img3);

S1=[];
S2=[];
S3=[];

N1=0;
N2=0;
N3=0;

A1=0;
A2=0;
A3=0;

%mes_threshold
thres=20;
if(M1(1,1)==0)
  S1=[0,0];
else
  S1=[M1(1,2)/M1(1,1), M1(2,1)/M1(1,1)];
  N1=sum(sum(img1>thres));
  A1=M1(1,1)/N1;
endif

if(M2(1,1)==0)
  S2=[0,0];
else
  S2=[M2(1,2)/M2(1,1), M2(2,1)/M2(1,1)];
  N2=sum(sum(img2>thres));
  A2=M2(1,1)/N2;
endif

if(M3(1,1)==0)
  S3=[0,0];
else
  S3=[M3(1,2)/M3(1,1), M3(2,1)/M3(1,1)];
  N3=sum(sum(img3>thres));
  A3=M3(1,1)/N3;
endif

%%
joint_pinch_closed=[1.57088 0.0513372 0.26181 -1.57082 -1.5708 0.0458558 0.261907];
%joint_spherical_closed=[1.04715 -0.261758 0.960078 -0.261554 0.960078 -0.261554 0.959965];%1.04715 -0.261758 0.960078 -0.261554 0.960078 -0.261554 0.959965

joint_spherical_closed=[1.0472 -0.488692 0.837758 -0.488692 0.837758 -0.488692 0.837758];

joint_sq_diff=0;
if(data_score(1,1)==3)
joint_sq_diff = sum(abs(data_kin - joint_spherical_closed))*180/pi;
else
joint_sq_diff = sum(abs(data_kin - joint_pinch_closed))*180/pi;
endif
joint_sq_diff;
%%%%%%%%%%%%%%%
A_selected = [data_score joint_sq_diff S1 A1 S2 A2 S3 A3];
A_raw = [data_score data_kin tactile_f1 tactile_f2 tactile_f3];
Label=data_success;

endfunction