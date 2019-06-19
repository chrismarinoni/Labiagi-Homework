
# Homework 5 - Interactive Graphic - PointCloud Library
### Exercise requests
- Read a sequence of ordered pairs of images (RGB+Depth images) and save the associated point cloud  with colors and surface normals on .pcd files (e.g.  cloud_001.pcd)
	- individual scene datasets can be found at the following address http://rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes/
- Then, read sequentially each .pcd file and:
	- align the current point cloud with the previous one by using Generalized ICP
	- save the cloud with its global transformation  (either transforming directly the cloud or using the sensor_origin and sensor_orientation parameter provided in the point cloud object)
	- apply a voxelization to the total point cloud (necessary  to reduce the dimension in terms of bytes) and visualize it  so that the entire scene reconstructed is shown
    - apply an additional filter of your choice to the point cloud.
- When everything of the two previous points seems working, acquire a dataset (using rosbag) throw a Kinect/Xtion of your face for both depth and RGB. Alternatively use the ROSbag at http://www.dis.uniroma1.it/~bloisi/didattica/RobotProgramming/face.bag
    - Visualize the bag in 3D in the PCLviewer

### First considerations
At the second point of the exercise we need to use one point cloud as reference and, sequentially, read the following .pcd files, and use the reference PCD as target for the GICP application.
The third request of the exercise is very generic. In the solution proposed here it was decided to use a special node that presents parts of code already used for the execution of the previous points.
So, we have four nodes in total:
- _bartolo_node_ implements the first request. Change the filepath to make it working on your pc. 
- _bartolo_gicp_node_ takes the reference PCD file and then applies sequentially the GICP algorithm using other PCD files
- _bartolo_rosbag_node_ acquires datas coming from the ROSbag and shows them
- _bartolo_visualizer_ shows a PCD file
### To run the exercise
The execution will be divided into three part, as for the requests.
- `rosrun bartolo bartolo_node` (it will start to load images and to create the related PCD files. Click Ctrl+C when you want to stop the process)
- `rosrun bartolo bartolo_gicp_node [NUM_PCD] [PATH]` where [NUM_PCD] in the number of PCD files to process with GICP and [PATH] is their directory. To visualize the result `rosrun bartolo bartolo_visualizer [NAMEFILE & ITS PATH]`. Note that the default path is added to "./".
- `rosbag play face.bag` and in a new terminal `rosrun bartolo bartolo_rosbag_node`
### Note
The code could have been more modular to improve readability and reduce redundancy. In these exercises I preferred, for reasons of time, to focus on the functioning of the code rather than on its form.
The PDF file "Hints Homework" contains some hints provided by the professor.

