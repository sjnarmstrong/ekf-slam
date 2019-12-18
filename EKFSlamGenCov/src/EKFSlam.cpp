/*
 * EKFSlam.cpp
 *
 *  Created on: 03 Nov 2018
 *      Author: sholto
 */

#include "EKFSlam.hpp"

EKFSlam::EKFSlam() {

	this->LandMarkStateVector = Eigen::VectorXd(MAX_LANDMARK_SIZE);
	this->LandMarkStateVariance = Eigen::MatrixXd::Zero(MAX_LANDMARK_SIZE, MAX_LANDMARK_SIZE);
	this->PosLandMarkRowStateVariance = Eigen::MatrixXd::Zero(POSITION_VECTOR_SIZE, MAX_LANDMARK_SIZE);
	this->PosLandMarkColStateVariance = Eigen::MatrixXd::Zero(MAX_LANDMARK_SIZE, POSITION_VECTOR_SIZE);

	this->PosState = Eigen::VectorXd::Zero(POSITION_VECTOR_SIZE);
	this->PosStateVariance = Eigen::MatrixXd::Zero(POSITION_VECTOR_SIZE, POSITION_VECTOR_SIZE);

	this->Jf = Eigen::MatrixXd::Identity(POSITION_VECTOR_SIZE, POSITION_VECTOR_SIZE);
	this->Q = Eigen::MatrixXd::Identity(POSITION_VECTOR_SIZE, POSITION_VECTOR_SIZE);

	//this->Q.block(0,0,3,3) *= 1.5;
	//this->Q.block(3,3,3,3) *= 20;
	this->Q.block(0,0,3,3) *= 0.9;
	this->Q.block(3,3,3,3) *= 16;

	this->feature_handler = cv::xfeatures2d::SURF::create(100,5,4,true);
	//this->feature_handler_detector = cv::xfeatures2d::SURF::create(300,5,3,true);
	//this->feature_handler_detector = cv::xfeatures2d::SIFT::create(0, 4, 0.001, 5, 0.3);
	this->feature_matcher = cv::BFMatcher(cv::NORM_L2, false);

	this->LandmarkEndIndex = 0;

}

Eigen::Matrix3d EKFSlam::GetRMat(double sy, double cy, double sa, double ca, double sb, double cb){
	Eigen::Matrix3d yMtx, bMtx, rMtx;
	yMtx << ca, -sa, 0, 
			sa, ca, 0, 
			0, 0, 1;
	bMtx << cb, 0, sb, 
			0, 1, 0, 
			-sb, 0, cb;
	rMtx << 1, 0, 0, 
			0, cy, -sy, 
			0, sy, cy;

	return (yMtx*bMtx*rMtx).cast<double>();
}

Eigen::Matrix3d EKFSlam::GetRMat(){
	double roll = this->PosState(3);
	double yaw = this->PosState(4);
	double pitch = this->PosState(5);

    double sy = std::sin(roll);
    double cy = std::cos(roll);

    double sa = std::sin(yaw);
    double ca = std::cos(yaw);

    double sb = std::sin(pitch);
    double cb = std::cos(pitch);

	return this->GetRMat(sy, cy, sa, ca, sb, cb);
}

void EKFSlam::GetJRxyz(Eigen::Matrix3d &Rx, Eigen::Matrix3d &Ry, Eigen::Matrix3d &Rz, Eigen::Matrix3d &R, double roll, double yaw, double pitch){

    double sy = std::sin(roll);
    double cy = std::cos(roll);

    double sa = std::sin(yaw);
    double ca = std::cos(yaw);

    double sb = std::sin(pitch);
    double cb = std::cos(pitch);

    Rx << 0, -sa * cb, -ca * sb,
        ca * sb * cy + sa * sy, -sa * sb * sy - ca * cy, ca * cb * sy,
        -ca * sb * sy + sa * cy, -sa * sb * cy + ca * sy, ca * cb * cy;
    Ry << 0, ca * cb, -sa * sb,
        sa * sb * cy - ca * sy, ca * sb * sy - sa * cy, sa * cb * sy,
        -sa * sb * sy - ca * cy, ca * sb * cy + sa * sy, sa * cb * cy;
    Rz << 0, 0, -cb,
        cb * cy, 0, -sb * sy,
        -cb * sy, 0, -sb * cy;
	R << this->GetRMat(sy, cy, sa, ca, sb, cb);
}

void EKFSlam::GetJRxyz(Eigen::Matrix3d &Rx, Eigen::Matrix3d &Ry, Eigen::Matrix3d &Rz, Eigen::Matrix3d &R){
	double roll = this->PosState(3);
	double yaw = this->PosState(4);
	double pitch = this->PosState(5);

	return this->GetJRxyz(Rx, Ry, Rz, R, roll, yaw, pitch);
}

void EKFSlam::reprojectPointWithSigmaInterval(double depth, const cv::Point2f &pt, Eigen::Vector3d &pt3d, Eigen::Vector3d &var3d){
	double Z = depth / SCALING_FACTOR;
	double Zp1 = (depth + 1.0) / SCALING_FACTOR;

	double n_x = (pt.x-CAMERA_C_X)/ CAMERA_FOCAL_LENGTH;
	double n_y = (pt.y-CAMERA_C_Y)/ CAMERA_FOCAL_LENGTH;
	double X = n_x*Z;
	double Y = n_y*Z;

	double sigma_x = std::abs(n_x*Zp1-X)/2.0;
	double sigma_y = std::abs(n_y*Zp1-Y)/2.0;
	double sigma_z = std::abs(Zp1-Z)/2.0;
	pt3d << X, Y, Z;
	var3d << sigma_x*sigma_x, sigma_y*sigma_y, sigma_z*sigma_z;
}

void EKFSlam::addPointToCurrentWorld(double depth, const cv::Point2f &pt, const cv::Mat descriptorRow, 
		const Eigen::Matrix3d &R, const Eigen::Vector3d &tvec, double offsetVar){


	Eigen::Vector3d pt3d, var3d;
	this->reprojectPointWithSigmaInterval(depth, pt, pt3d, var3d);
	pt3d = R*pt3d+tvec;

	Eigen::Matrix3d observationVariance;
	observationVariance << var3d(0)+offsetVar+CURRENT_STATE_VARIANCE_MULTIPLE_ADD*this->PosStateVariance(0,0), 0, 0, 0, var3d(1)+offsetVar+CURRENT_STATE_VARIANCE_MULTIPLE_ADD*this->PosStateVariance(1,1), 0, 0, 0, var3d(2)+offsetVar+CURRENT_STATE_VARIANCE_MULTIPLE_ADD*this->PosStateVariance(2,2);

	this->LandMarkStateVector.segment(this->LandmarkEndIndex, LANDMARK_LENGTH) << pt3d;
	this->LandMarkStateVariance.block(this->LandmarkEndIndex, this->LandmarkEndIndex,LANDMARK_LENGTH,LANDMARK_LENGTH) = 
		this->PosStateVariance.block(0,0,LANDMARK_LENGTH,LANDMARK_LENGTH)+observationVariance;

	//this->worldDescriptors.push_back();
	this->worldDescriptors.push_back(descriptorRow);

	this->LandmarkEndIndex+=3;

}

void EKFSlam::initialise(const cv::Mat &img, const cv::Mat &depth_img){

	cv::Mat descriptors;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<std::vector< cv::DMatch >> matches;

	//this->feature_handler_detector->detect(img,keypoints);
	//this->feature_handler_descriptor->compute(img, keypoints, descriptors);

	this->feature_handler->detectAndCompute(img, cv::noArray(),keypoints,descriptors);
    this->feature_matcher.knnMatch( descriptors, descriptors, matches, 2 );

	Eigen::Matrix3d R = this->GetRMat();
	Eigen::Vector3d tvec = this->PosState.segment(0,3);

	int max_to_add = 50;

	foreach(const std::vector<cv::DMatch> match, matches){
		if (match[1].distance > CLOSEST_CANDIDATE_THRESH_FF && match[0].trainIdx == match[0].queryIdx) {
			cv::KeyPoint kp = keypoints[match[0].trainIdx];
			//std::cout << kp.response << "  thresh:  " << MIN_CANDIDATE_THRESH_FF <<"\n";
			if (kp.response < MIN_CANDIDATE_THRESH_FF){continue;}

			double depth = depth_img.at<uint16_t>(std::round(kp.pt.y), std::round(kp.pt.x));
			if (depth == 0) {continue;}
			this->addPointToCurrentWorld(depth, kp.pt, descriptors.row(match[0].queryIdx), R, tvec);
			if (max_to_add-- <=0){
				std::cout<<"init max reached\n\n\n";
				break;
			}
		}
	}
}

void EKFSlam::DetectAndSeperateKeypoints(const cv::Mat &img, const cv::Mat &depth_img, 
		std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, 
		cv::Mat &cross_check_descriptors, 
		std::vector<int> &offset_rw_obs_ind, std::vector<Eigen::Vector3d> &TD_Vars, std::vector<int> &cand_ind,
		std::vector<Eigen::Vector3d> &TD_Pts, std::vector<uint16_t> &cand_dv, std::vector<double> &match_distance){
	
	std::vector<std::vector< cv::DMatch >> matches;
	Eigen::Vector3d pt3d, var3d, tvec, hold_rw_point;
	Eigen::Matrix3d R;
	tvec << this->PosState.segment(0,3);
	R << this->GetRMat();

	double threshhold = 24*(this->PosStateVariance(0,0)+this->PosStateVariance(1,1)+this->PosStateVariance(2,2))+0.040;
	//double threshhold = this->PosStateVariance(0,0)*this->PosStateVariance(0,0)+
	//	this->PosStateVariance(1,1)*this->PosStateVariance(1,1)+this->PosStateVariance(2,2)*this->PosStateVariance(2,2);
	this->feature_handler->detectAndCompute(img, cv::noArray(),keypoints,descriptors);
	std::cout << "thresh is: "<<threshhold<<"\n";
	//this->feature_handler_detector->detect(img,keypoints);
	if (keypoints.size()<MIN_KEYPOINTS){return;}
	//this->feature_handler_descriptor->compute(img, keypoints, descriptors);
    this->feature_matcher.knnMatch( descriptors, this->worldDescriptors, matches, 2 );
	if (matches.size() < MIN_KEYPOINTS){return;}

	foreach(const std::vector<cv::DMatch> match, matches){
		//std::cout << "Match dist " << match[0].distance <<"\n\n";
		if (match[0].distance < MAX_MATCH_THRESH && 
			match[0].distance<match[1].distance*CLOSEST_MATCH_THRESH_RATIO &&
			match[0].distance+CLOSEST_MATCH_THRESH<match[1].distance) {
			cv::KeyPoint kp = keypoints[match[0].queryIdx];
			double depth = depth_img.at<uint16_t>(std::round(kp.pt.y), std::round(kp.pt.x));
			if (depth == 0) {continue;}

			this->reprojectPointWithSigmaInterval(depth, kp.pt, pt3d, var3d);

			int rw_index = LANDMARK_LENGTH*match[0].trainIdx;
			//double dist = (R*pt3d+tvec - this->LandMarkStateVector.segment(rw_index, LANDMARK_LENGTH)).squaredNorm();
			double dist = (this->LandMarkStateVector.segment(rw_index, LANDMARK_LENGTH)-tvec).squaredNorm();
			dist -= pt3d.squaredNorm();
			if (std::abs(dist) > threshhold){
				std::cout << "Feature discarded due to its distance from the matched point\n Dist was: "<<dist<<"\n";
				continue;
			}
			TD_Pts.push_back(pt3d);
			TD_Vars.push_back(var3d);
			match_distance.push_back(match[0].distance);

			offset_rw_obs_ind.push_back(rw_index);
		} else if(match[0].distance > CLOSEST_CANDIDATE_THRESH){
			cv::KeyPoint kp = keypoints[match[0].queryIdx];
			if (kp.response < MIN_CANDIDATE_THRESH){continue;}
			double depth = depth_img.at<uint16_t>(std::round(kp.pt.y), std::round(kp.pt.x));
			if (depth == 0) {continue;}

			cand_dv.push_back(depth);
			cand_ind.push_back(match[0].queryIdx);
			cross_check_descriptors.push_back(descriptors.row(match[0].queryIdx));
		}
	}


}

void EKFSlam::PredictionStep(double deltaTime){
	double dt_sq = (deltaTime*deltaTime);
	this->PosStateVariance += this->Q*dt_sq; 
	this->max_squared_dist_to_travel += dt_sq*DELTA_DIST_THRESH_SQ;
}


void EKFSlam::getSmallerStateVectors(Eigen::MatrixXd &Pk, Eigen::MatrixXd &Rk, Eigen::MatrixXd &Jh, Eigen::VectorXd &relevantX, Eigen::VectorXd &ObservedValues, const Eigen::Vector3d &tvec,
		const std::vector<int> &offset_rw_obs_ind, const std::vector<Eigen::Vector3d> &TD_Vars, const std::vector<Eigen::Vector3d> &TD_Pts, const std::vector<double> &match_distance){
	
	Eigen::Matrix3d Rx, Ry, Rz, Rt, R;
	this->GetJRxyz(Rx, Ry, Rz, R);
	Rt = R.transpose();

	int number_of_observations = offset_rw_obs_ind.size();
	
	Pk.block(0,0,POSITION_VECTOR_SIZE,POSITION_VECTOR_SIZE) << this->PosStateVariance;

	int obs_pt_ind=0;

	relevantX.segment(0,6) << this->PosState;

	for (int loop = 0; loop < number_of_observations; loop++){
		int state_offset_obs_pt_ind = obs_pt_ind+POSITION_VECTOR_SIZE;
		int startPointInState=offset_rw_obs_ind[loop];

		Pk.block(0, state_offset_obs_pt_ind, POSITION_VECTOR_SIZE, LANDMARK_LENGTH) << this->PosLandMarkRowStateVariance.block(0,startPointInState,POSITION_VECTOR_SIZE,LANDMARK_LENGTH);
		Pk.block(state_offset_obs_pt_ind, 0,LANDMARK_LENGTH, POSITION_VECTOR_SIZE)  << this->PosLandMarkColStateVariance.block(startPointInState,0,LANDMARK_LENGTH,POSITION_VECTOR_SIZE);

		int current_i2=POSITION_VECTOR_SIZE;
		for (int loop2 = 0; loop2 < number_of_observations; loop2++){
			Pk.block(state_offset_obs_pt_ind, current_i2,LANDMARK_LENGTH,LANDMARK_LENGTH) << this->LandMarkStateVariance.block(startPointInState,offset_rw_obs_ind[loop2],LANDMARK_LENGTH,LANDMARK_LENGTH);
			current_i2+=LANDMARK_LENGTH;
		}

		Eigen::Vector3d pt3d, var3d;
		var3d = TD_Vars[loop];
		pt3d = TD_Pts[loop];

		double mach_score_certainty = match_distance[loop]*1e-3;

		Rk(obs_pt_ind, obs_pt_ind) = var3d(0)+mach_score_certainty;
		Rk(obs_pt_ind+1, obs_pt_ind+1) = var3d(1)+mach_score_certainty;
		Rk(obs_pt_ind+2, obs_pt_ind+2) = var3d(2)+mach_score_certainty;

		relevantX.segment(state_offset_obs_pt_ind, LANDMARK_LENGTH) <<  this->LandMarkStateVector.segment(startPointInState,LANDMARK_LENGTH);
		ObservedValues.segment(obs_pt_ind, LANDMARK_LENGTH) <<  pt3d;

		Jh.block(obs_pt_ind, 0, 3, 3) << -Rt;
		Jh.block(obs_pt_ind, state_offset_obs_pt_ind, 3, 3) << Rt;
		obs_pt_ind+=LANDMARK_LENGTH;
	}
}



void EKFSlam::recalculateRelevantV(const Eigen::VectorXd &relevantX, const Eigen::VectorXd &ObservedValues, Eigen::VectorXd &ek, Eigen::MatrixXd &Jh){

	Eigen::Matrix3d Rx, Ry, Rz, Rt, R;
	this->GetJRxyz(Rx, Ry, Rz, R, relevantX(3), relevantX(4), relevantX(5));
	Rt = R.transpose();

	for (int loop = 0; loop < ObservedValues.size(); loop+=LANDMARK_LENGTH){

		Eigen::Vector3d world_ponts = Eigen::Vector3d();
		world_ponts << relevantX.segment(loop+POSITION_VECTOR_SIZE, LANDMARK_LENGTH);

		Eigen::Vector3d points_in_current_referance_frame = Rt*(world_ponts-relevantX.segment(0, 3));

		ek.segment(loop, 3) << ObservedValues.segment(loop, LANDMARK_LENGTH)-points_in_current_referance_frame;

		Jh.block(loop, 3, 3, 3) << Rx*(world_ponts(0)-relevantX(0)) +
								   Ry*(world_ponts(1)-relevantX(1)) + 
								   Rz*(world_ponts(2)-relevantX(2));
	}
	
}


void EKFSlam::UpdateStateAndVariance(const Eigen::VectorXd &StateUpdate, const Eigen::MatrixXd &VarianceUpdate, const std::vector<int> &offset_rw_obs_ind){
	int obs_pt_ind=0;
	int number_of_observations = offset_rw_obs_ind.size();

	this->PosState = StateUpdate.segment(0,POSITION_VECTOR_SIZE);
	this->PosStateVariance << VarianceUpdate.block(0, 0, POSITION_VECTOR_SIZE, POSITION_VECTOR_SIZE);
	for (int loop = 0; loop < number_of_observations; loop++){
		int state_offset_obs_pt_ind = obs_pt_ind+POSITION_VECTOR_SIZE;
		int startPointInState=offset_rw_obs_ind[loop];

		this->PosLandMarkRowStateVariance.block(0,startPointInState,POSITION_VECTOR_SIZE,LANDMARK_LENGTH) << VarianceUpdate.block(0, state_offset_obs_pt_ind, POSITION_VECTOR_SIZE, LANDMARK_LENGTH);
		this->PosLandMarkColStateVariance.block(startPointInState,0,LANDMARK_LENGTH,POSITION_VECTOR_SIZE) << VarianceUpdate.block(state_offset_obs_pt_ind, 0,LANDMARK_LENGTH, POSITION_VECTOR_SIZE);
		int current_i2=POSITION_VECTOR_SIZE;
		for (int loop2 = 0; loop2 < number_of_observations; loop2++){
			this->LandMarkStateVariance.block(startPointInState,offset_rw_obs_ind[loop2],LANDMARK_LENGTH,LANDMARK_LENGTH) << VarianceUpdate.block(state_offset_obs_pt_ind, current_i2,LANDMARK_LENGTH,LANDMARK_LENGTH);
			current_i2+=LANDMARK_LENGTH;
		}
		this->LandMarkStateVector.segment(startPointInState,3) = StateUpdate.segment(state_offset_obs_pt_ind, 3);
		obs_pt_ind+=LANDMARK_LENGTH;
	}

}

void EKFSlam::processImage(const cv::Mat &img, const cv::Mat &depth_img, double deltaTime){
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	cv::Mat cross_check_descriptors = cv::Mat::zeros(0, 128, CV_32F);
	std::vector<int> offset_rw_obs_ind, cand_ind;
	std::vector<uint16_t> cand_dv;
	std::vector<Eigen::Vector3d> matched_3D_Pts, matched_3D_Vars;
	std::vector<double> match_distance;

	this->PredictionStep(deltaTime);

	this->DetectAndSeperateKeypoints(img, depth_img, keypoints, descriptors, cross_check_descriptors, offset_rw_obs_ind, matched_3D_Vars, cand_ind, matched_3D_Pts, cand_dv, match_distance);

	if (offset_rw_obs_ind.size() < MIN_KEYPOINTS){
		std:: cout << "Not Enough Observed points \n\n\n\n\n\n\n\n\n";
		return;
	}


	//Create smaller state vector and variance
	double prev_error_norm = 1e9; 
	double curr_err_norm;
	int eq_obs_size = 3*offset_rw_obs_ind.size();
	Eigen::MatrixXd IdentityMat = Eigen::MatrixXd::Identity(eq_obs_size+POSITION_VECTOR_SIZE, eq_obs_size+POSITION_VECTOR_SIZE) ;

	Eigen::VectorXd relevantX = Eigen::VectorXd::Zero(eq_obs_size+POSITION_VECTOR_SIZE);
	Eigen::VectorXd ObservedValues = Eigen::VectorXd::Zero(eq_obs_size);
	Eigen::MatrixXd Jh = Eigen::MatrixXd::Zero(eq_obs_size, eq_obs_size+POSITION_VECTOR_SIZE);
	Eigen::MatrixXd Pk = Eigen::MatrixXd::Zero(eq_obs_size+POSITION_VECTOR_SIZE, eq_obs_size+POSITION_VECTOR_SIZE);
	Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(eq_obs_size, eq_obs_size);
	Eigen::VectorXd ek = Eigen::VectorXd(eq_obs_size);
	Eigen::VectorXd StateUpdate;
	Eigen::MatrixXd VarianceUpdate;
	Eigen::Vector3d tvec = this->PosState.segment(0,3);
	this->getSmallerStateVectors(Pk, Rk, Jh, relevantX, ObservedValues, tvec, offset_rw_obs_ind, matched_3D_Vars, matched_3D_Pts, match_distance);

	for (int iteration =0; iteration < MAX_REFINEMENT_ITERS; iteration++){
		this->recalculateRelevantV(relevantX, ObservedValues, ek, Jh);
		Eigen::MatrixXd PfJht = Pk*Jh.transpose();
		Eigen::MatrixXd mtxToInvert = Jh*PfJht + Rk;
		Eigen::MatrixXd KkT = (mtxToInvert.transpose().householderQr().solve(PfJht.transpose())).transpose();
		double solution_error = (KkT*mtxToInvert - PfJht).norm() / PfJht.norm();
		if (solution_error > 3e-4){
			return;
		}

		StateUpdate = KkT*ek;
		VarianceUpdate = (IdentityMat - (KkT*Jh))*Pk;
		relevantX += StateUpdate;
		Pk = VarianceUpdate;

		//std::cout << "Updated Pos Info: " << this->PosState.transpose() << "\n";
		curr_err_norm = ek.norm();
		std::cout << "error: " << curr_err_norm << "\n";
		//std::cout << "solnErr: " << solution_error << "\n";
		//std::cout << "lmVar: " << this->LandMarkStateVariance.block(offset_rw_obs_ind[0],offset_rw_obs_ind[0],12,12) << "\n";
		if (prev_error_norm<1.02*curr_err_norm){break;}
		//if (StateUpdate.norm()>4*std::sqrt(this->PosStateVariance.block(0,0,6,6).trace())){return;}
		prev_error_norm = curr_err_norm;
	}
	curr_err_norm /=eq_obs_size;
	std::cout << "Final normalised error: " << curr_err_norm << "\n";
	if (curr_err_norm>0.05){
		std::cout << "Error too high to add new features \n\n";
		return;
	}
	double deltadist = (this->PosState.segment(0,3) - relevantX.segment(0,3)).squaredNorm();
	if (deltadist>this->max_squared_dist_to_travel){
		std::cout << "I see the machine tried to warp through space. I will not allow this. \n\n";
		return;
	}
	this->max_squared_dist_to_travel = 0;
	this->UpdateStateAndVariance(relevantX, Pk, offset_rw_obs_ind);

	std::cout << "Updated Pos Info: " << this->PosState.transpose() << "\n";
	std::vector<std::vector< cv::DMatch >> matches_to_add;
	this->feature_matcher.knnMatch( cross_check_descriptors, cross_check_descriptors, matches_to_add, 2 );

	Eigen::Matrix3d R = EKFSlam::GetRMat();
	tvec = this->PosState.segment(0,3);

	int number_to_add = MAX_LANDMARKS_IN_FRAME - matched_3D_Pts.size();
	std::cout<<"Number to add is: "<<number_to_add<<"\n";

	if (number_to_add <0){return;}

	for (int loop = 0; loop < matches_to_add.size(); loop++){
		const std::vector<cv::DMatch> match = matches_to_add[loop];
		if (match[1].distance > CLOSEST_CANDIDATE_THRESH && match[0].trainIdx == match[0].queryIdx) {
			cv::KeyPoint kp = keypoints[cand_ind[loop]];
			if (kp.response < MIN_CANDIDATE_THRESH){continue;}

			double depth = cand_dv[loop];
			//std::cout << "score: " << kp.response <<"\n";
			if (depth < 10||depth>15000) {continue;}
			this->addPointToCurrentWorld(depth, kp.pt, cross_check_descriptors.row(match[0].queryIdx), R, tvec, CURRENT_STATE_VARIANCE_ADDITION_ADD);

			if (--number_to_add<=0){
				std::cout<<"Max number added\n";
				break;
			}
		}
	}

}
