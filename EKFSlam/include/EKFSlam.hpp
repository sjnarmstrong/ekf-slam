/*
 * EKFSlam.h
 *
 *  Created on: 03 Nov 2018
 *      Author: sholto
 */

#ifndef EKFSLAM_H_
#define EKFSLAM_H_

#if(defined SHOW_INPUT_IMAGES || defined SHOW_DETECTED_KEYPOINTS_IMAGES || defined SHOW_MATCHED_KEYPOINTS)
	#define RENDER_ANY_IMAGES
#endif

#define CAMERA_FOCAL_LENGTH 525
#define CAMERA_C_X 319.5
#define CAMERA_C_Y 239.5
//#define SCALING_FACTOR 1.0 This value is for rosbags
#define SCALING_FACTOR 5000.0 
#define MIN_KEYPOINTS 12

#define LANDMARK_LENGTH 3
#define POSITION_VECTOR_SIZE 6
#define MAX_LANDMARK_SIZE 30000
//#define STATE_VECTOR_SIZE 1024

#define CLOSEST_MATCH_THRESH 0.15
#define CLOSEST_MATCH_THRESH_RATIO 0.68
#define MAX_MATCH_THRESH 0.24

#define MAX_LANDMARKS_IN_FRAME 40

#define CLOSEST_CANDIDATE_THRESH_FF 0.4
#define CLOSEST_CANDIDATE_THRESH 0.4
//#define MIN_CANDIDATE_THRESH_FF 0.0
//#define MIN_CANDIDATE_THRESH 0.0
#define MIN_CANDIDATE_THRESH_FF 300
#define MIN_CANDIDATE_THRESH 250

#define CURRENT_STATE_VARIANCE_MULTIPLE_ADD 3
#define CURRENT_STATE_VARIANCE_ADDITION_ADD 2e-4

#define MAX_REFINEMENT_ITERS 10
#define DELTA_DIST_THRESH_SQ 18

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


class EKFSlam {
public:
	EKFSlam();
	void processImage(const cv::Mat &img, const cv::Mat &depth_img, double deltaTime);
	void initialise(const cv::Mat &img, const cv::Mat &depth_img);


//These could be provate but left public for easy debugging

	cv::Mat_<_Float32> worldDescriptors = cv::Mat::zeros(0, 128, CV_32F);

	Eigen::VectorXd LandMarkStateVector;
	Eigen::MatrixXd LandMarkStateVariance;
	Eigen::MatrixXd PosLandMarkRowStateVariance;
	Eigen::MatrixXd PosLandMarkColStateVariance;

	int LandmarkEndIndex;

	Eigen::VectorXd PosState;
	Eigen::MatrixXd PosStateVariance;

	Eigen::MatrixXd Jf;
	Eigen::MatrixXd Q;

	//cv::Ptr<cv::xfeatures2d::SIFT> feature_handler_detector;
	//cv::Ptr<cv::xfeatures2d::SURF> feature_handler_detector;
	cv::Ptr<cv::xfeatures2d::SURF> feature_handler;
	cv::BFMatcher feature_matcher;

	double max_squared_dist_to_travel = 0;

	Eigen::Matrix3d GetRMat();
	Eigen::Matrix3d GetRMat(double sy, double cy, double sa, double ca, double sb, double cb);
	void GetJRxyz(Eigen::Matrix3d &Rx, Eigen::Matrix3d &Ry, Eigen::Matrix3d &Rz, Eigen::Matrix3d &R);
	void GetJRxyz(Eigen::Matrix3d &Rx, Eigen::Matrix3d &Ry, Eigen::Matrix3d &Rz, Eigen::Matrix3d &R, double roll, double yaw, double pitch);
	void addPointToCurrentWorld(double depth, const cv::Point2f &pt, const cv::Mat descriptorRow, const Eigen::Matrix3d &R, const Eigen::Vector3d &tvec, double offsetVar=0);
	void DetectAndSeperateKeypoints(const cv::Mat &img, const cv::Mat &depth_img, 
		std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, 
		cv::Mat &cross_check_descriptors, 
		std::vector<int> &offset_rw_obs_ind, std::vector<Eigen::Vector3d> &TD_Vars, std::vector<int> &cand_ind,
		std::vector<Eigen::Vector3d> &TD_Pts, std::vector<uint16_t> &cand_dv, std::vector<double> &match_distance);
	void PredictionStep(double deltaTime);
	void reprojectPointWithSigmaInterval(double depth, const cv::Point2f &pt, Eigen::Vector3d &pt3d, Eigen::Vector3d &var3d);
	void getSmallerStateVectors(Eigen::MatrixXd &Pk, Eigen::MatrixXd &Rk, Eigen::MatrixXd &Jh, Eigen::VectorXd &relevantX, Eigen::VectorXd &ObservedValues, const Eigen::Vector3d &tvec,
		const std::vector<int> &offset_rw_obs_ind, const std::vector<Eigen::Vector3d> &TD_Vars, const std::vector<Eigen::Vector3d> &TD_Pts, const std::vector<double> &match_distance);
	void recalculateRelevantV(const Eigen::VectorXd &relevantX, const Eigen::VectorXd &ObservedValues, Eigen::VectorXd &ek, Eigen::MatrixXd &Jh);

	void UpdateStateAndVariance(const Eigen::VectorXd &StateUpdate, const Eigen::MatrixXd &VarianceUpdate, const std::vector<int> &offset_rw_obs_ind);
private:

	const cv::Mat CAMERA_CALIB_MATRIX = (cv::Mat_<double>(3,3) << CAMERA_FOCAL_LENGTH, 0.0, CAMERA_C_X,
																0.0, CAMERA_FOCAL_LENGTH, CAMERA_C_Y,
																0.0, 0.0, 1.0);
	const Eigen::Matrix3d nI3 = -1.0*Eigen::Matrix3d::Identity();
	
};


#endif /* EKFSLAM_H_ */
