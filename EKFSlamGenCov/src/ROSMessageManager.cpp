#include "ROSMessageManager.hpp"

void ROSMessageManager::updateStateAndSendMessages(EKFSlam slamAlg, std::string timeStamp){

    tf::Vector3 origin(slamAlg.PosState(0), slamAlg.PosState(1), slamAlg.PosState(2));
    currentTransform.setOrigin(origin);

    double roll = slamAlg.PosState(3);
	double yaw = slamAlg.PosState(4);
	double pitch = slamAlg.PosState(5);

    qeig = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
    //std::cout << "Test2\n" << qeig.axis() << "\n" << qeig.angle()<<"\n";
    //std::cout << "Test\n" << qeig.toRotationMatrix() << "\n" << slamAlg.GetRMat()<<"\n";
    Eigen::Vector3f hold = qeig.axis();
    q.setRotation(tf::Vector3(hold(0),hold(1),hold(2)), qeig.angle());
    this->currentTransform.setRotation(q);
    br.sendTransform(tf::StampedTransform(currentTransform, ros::Time::now(), "world", "CameraPoseBeforeRefine"));
    std::cout << "current TF"   << " " << this->currentTransform.getOrigin().getX() 
                                << " " << this->currentTransform.getOrigin().getY() 
                                << " " << this->currentTransform.getOrigin().getZ() 
                                << " " << this->currentTransform.getRotation().getX() 
                                << " " << this->currentTransform.getRotation().getY() 
                                << " " << this->currentTransform.getRotation().getZ() 
                                << " " << this->currentTransform.getRotation().getW()  <<std::endl;
    this->writeTransform(timeStamp);

}



void ROSMessageManager::initOutputFile(std::string filename, std::string forTag){
    this->output_file.open(filename);
    this->output_file << "# ground truth trajectory"<<std::endl;
    this->output_file << "# file: '"<<forTag<<"'"<<std::endl;
    this->output_file << "# timestamp tx ty tz qx qy qz qw"<<std::endl;
}

void ROSMessageManager::writeTransform(std::string timeStamp){
    this->output_file << timeStamp  << " " << this->currentTransform.getOrigin().getX() 
                                    << " " << this->currentTransform.getOrigin().getY() 
                                    << " " << this->currentTransform.getOrigin().getZ() 
                                    << " " << this->currentTransform.getRotation().getX() 
                                    << " " << this->currentTransform.getRotation().getY() 
                                    << " " << this->currentTransform.getRotation().getZ() 
                                    << " " << this->currentTransform.getRotation().getW()  <<std::endl;
}
void ROSMessageManager::closeFile(){
    this->output_file.close();
}


