#include <iostream>
#include "parseFilenameTexts.hpp"
#include "EKFSlam.hpp"
#include "ROSMessageManager.hpp"

#include <opencv2/opencv.hpp>
#include <boost/date_time.hpp>

#define SHOW_OUTPUT_TO_ROS

using namespace std;
using namespace cv;

class TimeStructure_old{
public:
	int day, hour;
	double minutes;
	string fullTimestamp;
	inline TimeStructure_old(const string &timestamp){
		this->fullTimestamp = timestamp;
		this->day = stoi(timestamp.substr(0,2));
		this->hour = stoi(timestamp.substr(6,2));
		this->minutes = stod(timestamp.substr(8));
	}

	inline double getDifference(TimeStructure_old other){
		double diff_ms = (other.day - this->day) * 24.0;
		diff_ms = (other.hour - this->hour + diff_ms) * 60.0;
		return (other.minutes - this->minutes + diff_ms) * 60.0;
	}
	
};

class TimeStructure{
public:
	double double_time_v;
	string fullTimestamp;
	inline TimeStructure(const string &timestamp){
		this->fullTimestamp = timestamp;
		this->double_time_v = stod(timestamp);
	}

	inline double getDifference(TimeStructure other){
		return (other.double_time_v - this->double_time_v);
	}
	
};

int main ( int argc, char **argv )
{

    ros::init(argc, argv, std::string("EKFSlam"));
	//string basepath = "../Datasets/rgbd_dataset_freiburg1_rpy";
	//string outputTrajPath = "traj_rpy_output.txt";
	string basepath = "../Datasets/rgbd_dataset_freiburg1_xyz";
	string outputTrajPath = "traj_xyz_output.txt";
	//string outputTrajPath = "traj_xyz_output.txt";
	//string basepath = "../Datasets/rgbd_dataset_freiburg1_desk";
	//string outputTrajPath = "traj_desk_output.txt";
	if (argc >1) {basepath = argv[1];}
	if (argc >2) {outputTrajPath = argv[2];}

	int lastSlashPos = basepath.find_last_of('/');
	string datasetName = basepath.substr(lastSlashPos+1);

	FilenameTextParser ftp = FilenameTextParser(basepath);
	string rgbFilename, dFilename, timestamp;

	Mat rgbImage;
	Mat dImage;
	EKFSlam slamAlg = EKFSlam();
	ROSMessageManager RMM(argc, argv);

	RMM.initOutputFile(outputTrajPath, datasetName);
	if (!ftp.getNextFilenames(rgbFilename, dFilename, timestamp)){
		cout<<"Could not load any images\n";
		return 0;
	}

	TimeStructure time_1(timestamp);

	rgbImage = imread(rgbFilename, CV_LOAD_IMAGE_COLOR);
	dImage = imread(dFilename, CV_LOAD_IMAGE_GRAYSCALE|CV_LOAD_IMAGE_ANYDEPTH);

	slamAlg.initialise(rgbImage, dImage);
	RMM.updateStateAndSendMessages(slamAlg, timestamp);

	while (ftp.getNextFilenames(rgbFilename, dFilename, timestamp)){
    	rgbImage = imread(rgbFilename, CV_LOAD_IMAGE_COLOR);
    	dImage = imread(dFilename, CV_LOAD_IMAGE_GRAYSCALE|CV_LOAD_IMAGE_ANYDEPTH);

		TimeStructure time_2(timestamp);
		std::cout <<"delta T:"<<time_1.getDifference(time_2);

		slamAlg.processImage(rgbImage, dImage, time_1.getDifference(time_2));
		RMM.updateStateAndSendMessages(slamAlg, timestamp);


		imshow("rgbImage", rgbImage);
		imshow("dImage", dImage);
		waitKey(1);

		time_1=time_2;

	}
	RMM.closeFile();

	return 0;
}