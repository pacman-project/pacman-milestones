
#ifndef FORWARDKINEMATICS_H_
#define FORWARDKINEMATICS_H_


#include <vector>
#include <opencv2/core/core.hpp>


typedef std::vector<cv::Matx44d> Table_type;


class ForwardKinematics {

public:
	ForwardKinematics();
	virtual ~ForwardKinematics();

    void setGlobalTransformation(const cv::Matx44d & globalTrafo);
	/*!
	 * Initializes transformation matrices
	 * \param all_angles - Angles [phi0 .. phi6] in Radians
	 */
	void setAngles(std::vector<double> &all_angles);

	/*!
	 * Computes transformation matrix of taxel(_,y) on matrix m
	 * \param m - Matrix index
	 * \param y - y-coordinate on sensor matrix (only needed for distal matrix)
	 * \return Transformation matrix relative to hand's origin
	 */
	cv::Matx44d computeTransformationMatrixTaxelXYZ(int m, int y);

	/*!
	 * Computes transformation matrix of position(_,y) on matrix m
	 * \param m - Matrix index
	 * \param y - y-coordinate on sensor matrix (only needed for distal matrix)
	 * \return Transformation matrix relative to hand's origin
	 */
	cv::Matx44d computeTransformationMatrixPointOnSensorPlaneXYZ(int m, double y);

	/*!
	 * Computes cartesian coordinates (in mm) of taxel(x,y) on matrix m
	 * \param m - Matrix index
	 * \param x - x-coordinate on sensor matrix
	 * \param y - y-coordinate on sensor matrix
	 * \return Position [x,y,z] relative to hand's origin
	 */

    cv::Matx44d getFrameLinkCenter(int link_nr);
    cv::Matx44d getLinkCenters(int link_nr);


	std::vector<double> GetTaxelXYZ(int m, int x, int y);

	/*!
	 * Computes cartesian coordinates (in mm) of position(x,y) on matrix m
	 * (0.0, 0.0) is center of top-left taxel(0,0)
	 * Meaningful values that actual lie on the sensor plane:
	 *   Proximal: ([-1.7, 18.7], [-1.7, 45.9])
	 *   Distal:   ([-1.7, 18.7], [-1.7, 42.5)
	 *
	 * \param m - Matrix index
	 * \param x - x-coordinate on sensor matrix
	 * \param y - y-coordinate on sensor matrix
	 * \return Position [x,y,z] relative to hand's origin
	 */
	std::vector<double> GetPointOnSensorPlaneXYZ(int m, double x, double y);

private:

    //! global offset trafo to account for endeffector offset, can be set to left or right arm

    cv::Matx44d global_offset;
    //cv::Matx44d global_offset_left; //left of point
	//! Taxel transformation matrices (relative to sensor matrix base frame)
	cv::Matx44d P_prox;
	Table_type P_dist;

	//! Transformation matrices (Denavit-Hartenberg)
	//! Finger 0
	cv::Matx44d T02_f0;
	cv::Matx44d T03_f0;

	//! Finger 1
	cv::Matx44d T02_f1;
	cv::Matx44d T03_f1;

	//! Finger 2
	cv::Matx44d T02_f2;
	cv::Matx44d T03_f2;


	inline double degToRad(double d) {
	    return d * M_PI/180.0;
	}


	/*!
	 *  Generates a generic transformation matrix from frame n-1 to frame n
	 *  Follows Denavit-Hartenberg convention:
	 * \param d_n      - Distance between \f$x_n\f$  and \f$x_n-1\f$ along \f$z_n-1\f$
	 * \param Theta_n  - Rotation about \f$z_n-1\f$ (in Radians)
	 * \param r_n      - Radius of rotation about \f$z_n-1\f$
	 * \param alpha_n  - Rotation about \f$x_n\f$ (in Radians)
	 * \param T        - Resulting transformation matrix
	 */
	void createTransformationMatrix(double d_n, double Theta_n, double r_n, double alpha_n, cv::Matx44d& T);



	/*!
	 * Transform Sensor matrix coordinates (x,y) to coordinate system O2 (x2, y2, z2)
	 *
	 * \return The proximal transformation matrix
	 */
	cv::Matx44d createTransformationMatrix_proximal();

	/*!
	 * Transform Sensor matrix coordinates (x,y) to coordinate system O3 (x3, y3, z3)
	 * \param y - y-coordinate on distal sensor matrix
	 *
	 *  \return The distal transformation matrix
	 */
	cv::Matx44d createTransformationMatrix_distal(double y);


	/*!
	 * Creates a table of transformation matrices for each proximal taxel
	 */
	void createTransformationTable_proximal();

	/*!
	 * Creates a table of transformation matrices for each distal taxel
	 */
	void createTransformationTable_distal();

	//--------------------------------------------------------------------
	// Public

	/*!
	 * Generates transformation matrices from the hand's origin to tip of Finger 0
	 * T02_f0 and T03_f0 correspond to the base frames of proximal and distal sensors
	 * \param phi0 - Axis 0: Rotational angle
	 * \param phi1 - Axis 1
	 * \param phi2 - Axis 2
	 */
	void createTransformationMatrices_f0(double phi0, double phi1, double phi2);

	/*!
	 * Generates transformation matrices from the hand's origin to tip of Finger 1
	 * T02_f1 and T03_f1 correspond to the base frames of proximal and distal sensors
	 * \param phi3 - Axis 3
	 * \param phi4 - Axis 4
	 */
	void createTransformationMatrices_f1(double phi3, double phi4);

	/*!
	 * Generates transformation matrices from the hand's origin to tip of Finger 2
	 * T02_f2 and T03_f2 correspond to the base frames of proximal and distal sensors
	 * \param phi0 - Axis 0: Rotational angle
	 * \param phi5 - Axis 5
	 * \param phi6 - Axis 6
	 */
	void createTransformationMatrices_f2(double phi0, double phi5, double phi6);


};

#endif /* FORWARDKINEMATICS_H_ */
