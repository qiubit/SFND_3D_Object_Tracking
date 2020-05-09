# SFND_3D_Object_Tracking Writeupp

This WRITEUP contains writeup addressing tasks of 3D Object Tracking project of Udacity's Sensor Fusion Nanodegree. Performance evaluation of feature detectors and descriptors is also listed.

### FP.1 Match 3D Objects
In order to match bounding boxes between images, for each keypoint a list of bounding boxes containing this keypoint is constructed. Then, for each `cv::DMatch` object, I add information to interal `std::unordered_map` object about all pairings between bounding boxes that contain keypoints on each end of the match. At the end, for each bounding box B1, we pair it with the bounding box B2, which had the most pairings (B1, B2) resulting from analyzing `cv::DMatch` objects.

### FP.2 Compute LIDAR-based TTC
The code closely follows one presented in lectures. The main difference is that xmin is defined as 5'th smallest x coordinate present in inputted LIDAR points. This choice was made, because after observing top-down LIDAR images, it can be seen that there is a cluster of points near the back of the car, with similar x coordinate, and about 2-3 points can be outliers in an analyzed dataset.

### FP.3 Associate Keypoint Correspondences with Bounding Boxes
For each `cv::DMatch` object, if given bounding box contains matched keypoint, the match is added to internal `kptMatches` field.

### FP.4 : Compute Camera-based TTC
The code closely follows the lecture.

### FP.5 : Performance Evaluation 1
The following chart illustrates TTC sequence reported using LIDAR measurements:

[LIDAR](dat/ttc_lidar_charts/lidar_ttc.png)

What's interesting is that the downward trend visible in LIDAR TTC chart is much less clear than the one resulting from Camera TTC measurements (shown in the next section). The reason might be that in case of computing Camera TTC, the velocity computation is implicit, while in case of LIDAR, we explicitly compute the velocity between two consecutive frames, to use it for TTC computation. That means that LIDAR TTC computation is very sensitive to violations of Constant Velocity movement model, which may occur when the driver is nearing a vehicle (which is the situation analyzed in the dataset).

The following analysis will try to show that contant velocity model is indeed violated in the dataset. By looking at the dataset it can be seen that LIDAR position measuremnts are pretty accurate - the clusters are clear and not noisy. Now, assuming constant velocity, the distance of the cluster corresponding to the car in front of the ego-car should decrease by the same amount between different frames, under the assumption of constant camera framerate.

Let's analyze the following sequence:

[1](dat/ttc_lidar_charts/lidar_0.png)
[2](dat/ttc_lidar_charts/lidar_1.png)
[3](dat/ttc_lidar_charts/lidar_2.png)

By looking at xmin difference, we can compute differences in x coordinate - they are 7.97-7.91=0.06m for the first pair, and 7.91-7.85=0.06m. So far so good, velocity seems constant.

But let's look at the following sequence:

[1](dat/ttc_lidar_charts/lidar_6.png)
[2](dat/ttc_lidar_charts/lidar_7.png)

This time, the x difference is 7.58-7.55=0.03m, so the velocity computed between those two frames will be twice lower than the velocity computed between previous frames. That means, the TTC will be overestimated, it is indeed the reason between a big bump on the LIDAR TTC chart presented at the beginning of this section.

### FP.6 : Performance Evaluation 2
The following charts illustrate TTC sequences reported for all possible detector/descriptor pairs:

[SHITOMASI](dat/ttc_cam_charts/SHITOMASI.png)
[HARRIS](dat/ttc_cam_charts/HARRIS.png)
[AKAZE](dat/ttc_cam_charts/AKAZE.png)
[BRISK](dat/ttc_cam_charts/BRISK.png)
[FAST](dat/ttc_cam_charts/FAST.png)
[ORB](dat/ttc_cam_charts/ORB.png)
[SIFT](dat/ttc_cam_charts/SIFT.png)

All charts, except for ORB (which might not be sufficient for the task), present a noisy downward trend, which present that with Constant Velocity movement model it is feasible to detect whether we are getting closer to vehicle in front of us. Performance of descriptors minimally affects pipeline's accuracy - the appropriate choice of detector is much more crucial. The clearest downward trend is presented on AKAZE chart, which suggests that AKAZE is the best detector for implementing colision detector.
