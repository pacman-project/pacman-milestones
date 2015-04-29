
#include <iostream>
#include <iomanip>
#include "forwardKinematics.h"

using namespace std;

// Note: Written for readability. Let's rely on the compiler's optimization

ForwardKinematics::ForwardKinematics() {

	// Create static transformation matrices
	createTransformationTable_proximal();
	createTransformationTable_distal();

    

    global_offset = cv::Matx44d::eye(); //now should be in the hand frame!
}

ForwardKinematics::~ForwardKinematics() { }

void ForwardKinematics::setGlobalTransformation(const cv::Matx44d & globalTrafo){
  double l0=0.017*1000;
  double lx=0.098*1000;
  
  global_offset=globalTrafo*cv::Matx44d( 1.0, 0.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0, 0.0,
                                 0.0, 0.0, 1.0, -l0+lx/2,
                                 0.0, 0.0, 0.0, 1.0 );
}


void ForwardKinematics::setAngles(std::vector<double> &all_angles) {

	// Create dynamic transformation matrices
	createTransformationMatrices_f0(all_angles[0], all_angles[1], all_angles[2]);
	createTransformationMatrices_f1(all_angles[3], all_angles[4]);
	createTransformationMatrices_f2(all_angles[0], all_angles[5], all_angles[6]);
}


void ForwardKinematics::createTransformationMatrix(double d_n, double Theta_n, double r_n, double alpha_n, cv::Matx44d& T) {

	T = cv::Matx44d( cos(Theta_n), -sin(Theta_n)*cos(alpha_n),  sin(Theta_n)*sin(alpha_n), r_n*cos(Theta_n),
			         sin(Theta_n),  cos(Theta_n)*cos(alpha_n), -cos(Theta_n)*sin(alpha_n), r_n*sin(Theta_n),
			         0.0,           sin(alpha_n),               cos(alpha_n),              d_n,
			         0.0,           0.0,                        0.0,                       1.0 );
}


void ForwardKinematics::createTransformationMatrices_f0(double phi0, double phi1, double phi2) {

	double l1 = 86.5;
	double l2 = 68.5;
	double h = 17.0;

	// Denavit-Hartenberg parameters
	double l[3] = { h, l1, l2 }; // link lengths
	double distance[3] = { l[0], 0.0, 0.0 };
	double Theta[3] = { -phi0, phi1-degToRad(90.0), phi2 };
	double r[3] = { 0.0, l[1], l[2]};
	double alpha[3] =  { degToRad(-90.0), 0.0, 0.0 };

	// Finger transformation matrices
	cv::Matx44d T01_f0;
	cv::Matx44d T12_f0;
	cv::Matx44d T23_f0;
	createTransformationMatrix(distance[0], Theta[0], r[0], alpha[0], T01_f0);
	createTransformationMatrix(distance[1], Theta[1], r[1], alpha[1], T12_f0);
	createTransformationMatrix(distance[2], Theta[2], r[2], alpha[2], T23_f0);

	// Apply finger offset (equilateral triangle relations)
	double tx = 19.05255888325765; // 1/3 * h_tiangle = 1/3 * (1/2*sqrt(3)*d);
	double ty = -33.0; // -0.5 * d;
	double tz = 0.0;

	// Finger frame
	cv::Matx44d T00_f0 = cv::Matx44d( -1.0, 0.0, 0.0, tx,
			                           0.0, -1.0, 0.0, ty,
			                           0.0, 0.0, 1.0,  tz,
			                           0.0, 0.0, 0.0, 1.0);

	T01_f0 = T00_f0 * T01_f0; // Frame after rotational joint
	T02_f0 = T01_f0 * T12_f0; // Base frame for proximal sensor matrix
	T03_f0 = T02_f0 * T23_f0; // Base frame for distal sensor matrix (End effector, i.e. fingertip)
}

void ForwardKinematics::createTransformationMatrices_f1(double phi3, double phi4) {

	double l1 = 86.5;
	double l2 = 68.5;
	double h = 17.0;

	// Denavit-Hartenberg parameters
	double l[3] = { h, l1, l2 }; // link lengths
	double distance[3] = { l[0], 0.0, 0.0 };
	double Theta[3] = { degToRad(180.0), phi3-degToRad(90.0), phi4 }; // phi0 is stiffened
	double r[3] = { 0.0, l[1], l[2]};
	double alpha[3] =  { degToRad(-90.0), 0.0, 0.0 };

	// Finger transformation matrices
	cv::Matx44d T01_f1;
	cv::Matx44d T12_f1;
	cv::Matx44d T23_f1;
	createTransformationMatrix(distance[0], Theta[0], r[0], alpha[0], T01_f1);
	createTransformationMatrix(distance[1], Theta[1], r[1], alpha[1], T12_f1);
	createTransformationMatrix(distance[2], Theta[2], r[2], alpha[2], T23_f1);

	// Apply finger offset (Equilateral triangle relations)
	double tx = -38.1051177665153; // -2/3 * h_tiangle = -2/3 * (1/2*sqrt(3)*d);
	double ty = 0.0;
	double tz = 0.0;

	// Finger frame
	cv::Matx44d T00_f1 = cv::Matx44d( -1.0, 0.0, 0.0, tx,
	                                   0.0, -1.0, 0.0, ty,
	                                   0.0, 0.0, 1.0,  tz,
	                                   0.0, 0.0, 0.0, 1.0);

	T01_f1 = T00_f1 * T01_f1; // Frame after rotational joint
	T02_f1 = T01_f1 * T12_f1; // Base frame for proximal sensor matrix
	T03_f1 = T02_f1 * T23_f1; // Base frame for distal sensor matrix (End effector, i.e. fingertip)
}


void ForwardKinematics::createTransformationMatrices_f2(double phi0, double phi5, double phi6) {

	double l1 = 86.5;
	double l2 = 68.5;
	double h = 17.0;

	// Denavit-Hartenberg parameters
	double l[3] = { h, l1, l2 }; // link lengths
	double distance[3] = { l[0], 0.0, 0.0 };
	double Theta[3] = { phi0, phi5-degToRad(90.0), phi6 };
	double r[3] = { 0.0, l[1], l[2]};
	double alpha[3] =  { degToRad(-90.0), 0.0, 0.0 };

	// Finger transformation matrices
	cv::Matx44d T01_f2;
	cv::Matx44d T12_f2;
	cv::Matx44d T23_f2;
	createTransformationMatrix(distance[0], Theta[0], r[0], alpha[0], T01_f2);
	createTransformationMatrix(distance[1], Theta[1], r[1], alpha[1], T12_f2);
	createTransformationMatrix(distance[2], Theta[2], r[2], alpha[2], T23_f2);

	// Apply finger offset (Equilateral triangle relations)
	double tx = 19.05255888325765; // 1/3 * h_tiangle = 1/3 * (1/2*sqrt(3)*d);
	double ty = 33.0; // 0.5 * d;
	double tz = 0.0;

	// Finger frame
	cv::Matx44d T00_f2 = cv::Matx44d( -1.0, 0.0, 0.0, tx,
	                                   0.0, -1.0, 0.0, ty,
	                                   0.0, 0.0, 1.0,  tz,
	                                   0.0, 0.0, 0.0, 1.0);

	T01_f2 = T00_f2 * T01_f2; // Frame after rotational joint
	T02_f2 = T01_f2 * T12_f2; // Base frame for proximal sensor matrix
	T03_f2 = T02_f2 * T23_f2; // Base frame for distal sensor matrix (End effector, i.e. fingertip)
}

cv::Matx44d ForwardKinematics::getFrameLinkCenter(int m){

//Link nr = -1 = Basis of SDH,
//Link nr = 0 = Finger 0 proximal
//....
//Link nr = 6 = Finger 2 distal

//For now only get the end of the frame!!!
if(m==-1){
    return cv::Matx44d::eye();
}
else if(m==0){
    return global_offset*T02_f0;
}
else if(m==1){
    return global_offset*T03_f0;
}
else if(m==2){
    return global_offset*T02_f1;
}
else if(m==3){
    return global_offset*T03_f1;
}
else if(m==4){
    return global_offset*T02_f2;
}
else if(m==5){
    return global_offset*T03_f2;
}
else{
return cv::Matx44d::eye();
}


}

cv::Matx44d ForwardKinematics::getLinkCenters(int m){

//Link nr = -1 = Basis of SDH,
//Link nr = 0 = Finger 0 proximal
//....
//Link nr = 6 = Finger 2 distal

//For now only get the end of the frame!!!
if(m==-1){
    return cv::Matx44d::eye();
}
else if(m==0){

    double l2 = 86.5;
    cv::Matx44d offset = cv::Matx44d::eye();
    offset(0,3)=-l2/2;

    return global_offset*T02_f0 * offset;
}
else if(m==1){

    double l2 = 68.5;
    cv::Matx44d offset = cv::Matx44d::eye();
    offset(0,3)=-l2/2;

    return global_offset*T03_f0 * offset;
}
else if(m==2){

    double l2 = 86.5;
    cv::Matx44d offset = cv::Matx44d::eye();
    offset(0,3)=-l2/2;
    return global_offset*T02_f1 * offset;
}
else if(m==3){
    double l2 = 68.5;
    cv::Matx44d offset = cv::Matx44d::eye();
    offset(0,3)=-l2/2;
    return global_offset*T03_f1*offset;
}
else if(m==4){
    double l2 = 86.5;
    cv::Matx44d offset = cv::Matx44d::eye();
    offset(0,3)=-l2/2;
    return global_offset*T02_f2 * offset;
}
else if(m==5){
    double l2 = 68.5;
    cv::Matx44d offset = cv::Matx44d::eye();
    offset(0,3)=-l2/2;
    return global_offset*T03_f2*offset;
}
else{
return cv::Matx44d::eye();
}


}


cv::Matx44d ForwardKinematics::createTransformationMatrix_proximal() {

	double l2 = 86.5;
	double s1 = 17.5;
	double a = 4.1;
	double w = 6*3.4;
	double depth = 15.43;

	// Transformation within the sensor matrix
	double sx =  3.4;
	double sy = -3.4;
	double tx = 3.4/2.0 - w/2.0;
	double ty = 13*3.4;

	// Translation relative to O2
	double tx2 = -l2 + s1 + a;
	double ty2 = depth;

	return cv::Matx44d( sy, 0.0, 0.0, ty+tx2,
	                    0.0, 1.0, 0.0,  ty2,
	                    0.0, 0.0,  sx,   tx,
	                    0.0, 0.0, 0.0,  1.0);
}


cv::Matx44d ForwardKinematics::createTransformationMatrix_distal(double y) {

	double l3 = 68.5;
	double s3 = 17.5;
	double a = 4.95;
	double taxel_width = 3.4;
	double matrix_width = 6*3.4;
	double R = 60.0;
	double depth = 15.0;

	double sx, sy, tx, ty, tx3, ty3;

	if(y > 8.5) { // Planar part

		// Transformation within sensor matrix
		sx = taxel_width;
		sy = -taxel_width;
		tx = taxel_width/2.0 - matrix_width/2.0;
		ty = 12*3.4;

		// Translation relative to O3
		tx3 = -l3 + s3 + a;
		ty3 = depth;

	} else { // Curved part

		// Transformation within sensor matrix
		sx = taxel_width;
		sy = 1.0;
		tx = taxel_width/2.0 - matrix_width/2.0;
		ty = (R * sin( ((8.5-y)*3.4) / R )) + 3.5*3.4 - y;

		// Translation relative to O3
		tx3 = -l3 + s3 + a;
		ty3 = depth - (R - R * cos( ((8.5-y)*3.4) / R));
	}

	return cv::Matx44d(  sy, 0.0, 0.0, ty+tx3,
	                     0.0, 1.0, 0.0,  ty3,
	                     0.0, 0.0,  sx,   tx,
	                     0.0, 0.0, 0.0,  1.0);
}


void ForwardKinematics::createTransformationTable_proximal() {
	P_prox = createTransformationMatrix_proximal();
}


void ForwardKinematics::createTransformationTable_distal() {
	P_dist.resize(13);
	for(int y = 0; y < 13; y++) {
		P_dist[y] = createTransformationMatrix_distal(static_cast<double>(y));
	}
}

cv::Matx44d ForwardKinematics::computeTransformationMatrixTaxelXYZ(int m, int y) {
	cv::Matx44d T_total;

	// Combine dynamic transformation matrices of finger joints with static sensor matrix offset
	if(m == 0) { // Finger 0: Proximal
        T_total = T02_f0 * P_prox;
	}
	else if(m == 1) { // Finger 0: Distal
		T_total = T03_f0 * P_dist[y];
	}
	else if (m == 2) { // Finger 1: Proximal
		T_total = T02_f1 * P_prox;
	}
	else if(m == 3) { // Finger 1: Distal
		T_total = T03_f1 * P_dist[y];
	}
	else if (m == 4) { // Finger 2: Proximal
		T_total = T02_f2 * P_prox;
	}
	else if(m == 5) { // Finger 2: Distal
		T_total = T03_f2 * P_dist[y];
	}
	return T_total;
}

cv::Matx44d ForwardKinematics::computeTransformationMatrixPointOnSensorPlaneXYZ(int m, double y) {

	cv::Matx44d T_total;
	double y_prime = y/3.4;

	// Combine dynamic transformation matrices of finger joints with dynamic sensor matrix coordinates
	if(m == 0) { // Finger 0: Proximal
		T_total = T02_f0 * createTransformationMatrix_proximal();
	}
	else if(m == 1) { // Finger 0: Distal
		T_total = T03_f0 * createTransformationMatrix_distal(y_prime);
	}
	else if (m == 2) { // Finger 1: Proximal
		T_total = T02_f1 * createTransformationMatrix_proximal();
	}
	else if(m == 3) { // Finger 1: Distal
		T_total = T03_f1 * createTransformationMatrix_distal(y_prime);
	}
	else if (m == 4) { // Finger 2: Proximal
		T_total = T02_f2 * createTransformationMatrix_proximal();
	}
	else if(m == 5) { // Finger 2: Distal
		T_total = T03_f2 * createTransformationMatrix_distal(y_prime);
	}
	return T_total;
}

std::vector<double> ForwardKinematics::GetTaxelXYZ(int m, int x, int y) {

	cv::Matx44d T_total = computeTransformationMatrixTaxelXYZ(m, y);

	// Map cell index to point in 3D sensor matrix space
	cv::Vec4d p = cv::Vec4d(y, 0.0, x, 1.0);

	// Transform point
    cv::Vec4d p_transformed = global_offset * T_total * p;

	std::vector<double>coordinate(3);
	coordinate[0] = p_transformed[0];
	coordinate[1] = p_transformed[1];
	coordinate[2] = p_transformed[2];

	return coordinate;
}


std::vector<double> ForwardKinematics::GetPointOnSensorPlaneXYZ(int m, double x, double y) {

	cv::Matx44d T_total = computeTransformationMatrixPointOnSensorPlaneXYZ(m, y);

	double x_prime = x/3.4;
	double y_prime = y/3.4;

	// Map cell index to point in 3D sensor matrix space
	cv::Vec4d p = cv::Vec4d(y_prime, 0.0, x_prime, 1.0);

	// Transform point
    cv::Vec4d p_transformed = global_offset * T_total * p;

	std::vector<double>coordinate(3);
	coordinate[0] = p_transformed[0];
	coordinate[1] = p_transformed[1];
	coordinate[2] = p_transformed[2];

	return coordinate;
}

