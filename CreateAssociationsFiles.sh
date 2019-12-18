echo Deleting old associations files
rm Datasets/rgbd_dataset_freiburg1_desk/associations.txt Datasets/rgbd_dataset_freiburg1_xyz/associations.txt Datasets/rgbd_dataset_freiburg1_rpy/associations.txt
echo Adding new association files
rosrun rgbd_benchmark_tools associate.py Datasets/rgbd_dataset_freiburg1_desk/depth.txt Datasets/rgbd_dataset_freiburg1_desk/rgb.txt >> Datasets/rgbd_dataset_freiburg1_desk/associations.txt
rosrun rgbd_benchmark_tools associate.py Datasets/rgbd_dataset_freiburg1_xyz/depth.txt Datasets/rgbd_dataset_freiburg1_xyz/rgb.txt >> Datasets/rgbd_dataset_freiburg1_xyz/associations.txt
rosrun rgbd_benchmark_tools associate.py Datasets/rgbd_dataset_freiburg1_rpy/depth.txt Datasets/rgbd_dataset_freiburg1_rpy/rgb.txt >> Datasets/rgbd_dataset_freiburg1_rpy/associations.txt
