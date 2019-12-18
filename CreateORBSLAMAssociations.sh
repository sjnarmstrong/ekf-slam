echo Deleting old associations files
rm Datasets/rgbd_dataset_freiburg1_desk/associationsORB.txt Datasets/rgbd_dataset_freiburg1_xyz/associationsORB.txt Datasets/rgbd_dataset_freiburg1_rpy/associationsORB.txt
echo Adding new association files
rosrun rgbd_benchmark_tools associate.py Datasets/rgbd_dataset_freiburg1_desk/rgb.txt Datasets/rgbd_dataset_freiburg1_desk/depth.txt >> Datasets/rgbd_dataset_freiburg1_desk/associationsORB.txt
rosrun rgbd_benchmark_tools associate.py Datasets/rgbd_dataset_freiburg1_xyz/rgb.txt Datasets/rgbd_dataset_freiburg1_xyz/depth.txt >> Datasets/rgbd_dataset_freiburg1_xyz/associationsORB.txt
rosrun rgbd_benchmark_tools associate.py Datasets/rgbd_dataset_freiburg1_rpy/rgb.txt Datasets/rgbd_dataset_freiburg1_rpy/depth.txt >> Datasets/rgbd_dataset_freiburg1_rpy/associationsORB.txt