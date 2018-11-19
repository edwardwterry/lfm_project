#include <iostream>

#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs_types.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>
#include <swiftpro/position.h>
#include <swiftpro/SwiftproState.h>


#include <lfm/AprilTagDetection.h>
#include <lfm/AprilTagDetectionArray.h>
#include <lfm/Action.h>


#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstring>
