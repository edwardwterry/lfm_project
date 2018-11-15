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
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>

#include <lfm/AprilTagDetection.h>
#include <lfm/AprilTagDetectionArray.h>
#include "lfm.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>

static const std::string OPENCV_WINDOW = "Image window";

// class ImageConverter
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
// //   image_transport::Subscriber color_sub_;
// //   image_transport::Subscriber depth_sub_;  
//   image_transport::Publisher image_pub_;

// public:
//   ImageConverter()
//     : it_(nh_)
//   {
//     // Subscrive to input video feed and publish output video feed
//     // color_sub_ = it_.subscribe("/camera/color/image_raw", 1,
//     //   &ImageConverter::colorCb, this);
//     // depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
//     //   &ImageConverter::colorCb, this);
//     // image_pub_ = it_.advertise("/image_converter/output_video", 1);

//     // cv::namedWindow(OPENCV_WINDOW);
//   }

//   ~ImageConverter()
//   {
//     // cv::destroyWindow(OPENCV_WINDOW);
//   }

// };

class WorldCoords {
    public:
        void run();
        void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale);//, float clipping_dist);
        float get_depth_scale(rs2::device dev);
        rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
        bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
        std::map<int, Eigen::Vector3f> generateTagsInArmCoords();
        // void segmentByColor(cv::Mat & image, cv::Scalar target_color);
        void processTagCentersClbk(const std_msgs::Float32MultiArray& msg);
        void processTag3dCentersClbk(const lfm::AprilTagDetectionArray& msg);
        rs2_intrinsics createRs2Intrinsics();
        sensor_msgs::CameraInfo generateCalibrationData();
        bool calculateExtrinsics(const std::map<int, Eigen::Vector3f>& cam, 
                                                const std::map<int, Eigen::Vector3f>& arm, 
                                                Eigen::Matrix3f& R, 
                                                Eigen::Vector3f& t);

        ros::NodeHandle n;
        rs2::pipeline pipe;
        ros::Subscriber tag_centers_sub = n.subscribe("/apriltags2_ros_continuous_node/tag_pixel_centers", 1000, &WorldCoords::processTagCentersClbk, this);
        ros::Subscriber tag_centers_3d_sub = n.subscribe("/tag_detections", 1000, &WorldCoords::processTag3dCentersClbk, this);
        std::map<int, Eigen::Vector2f> tag_centers_pix_cam;
        std::map<int, Eigen::Vector3f> tag_centers_3d_cam;
        std::map<int, Eigen::Vector3f> tag_centers_3d_arm;
        sensor_msgs::CameraInfo info_msg;
        rs2_intrinsics rs2_intr;
        Eigen::Matrix3f R;
        Eigen::Vector3f t;
};

sensor_msgs::CameraInfo WorldCoords::generateCalibrationData()
{
  sensor_msgs::CameraInfo ci;

  ci.width = 640u;
  ci.height = 480u;

  // set distortion coefficients
  ci.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  ci.D.resize(5);
  ci.D[0] = 0.14570510511071208;
  ci.D[1] = -0.3424532038552255;
  ci.D[2] = -0.0012457073683988366;
  ci.D[3] = -0.002490918767386115;
  ci.D[4] = 0.0;

  // set camera matrix
  ci.K[0] = 634.3947866272065;
  ci.K[1] = 0.0;
  ci.K[2] = 321.55435855322696;
  ci.K[3] = 0.0;
  ci.K[4] = 635.9909775558393;
  ci.K[5] = 236.97478263483032;
  ci.K[6] = 0.0;
  ci.K[7] = 0.0;
  ci.K[8] = 1.0;

  // set rectification matrix
  ci.R[0] = 1.0;
  ci.R[1] = 0.0;
  ci.R[2] = 0.0;
  ci.R[3] = 0.0;
  ci.R[4] = 1.0;
  ci.R[5] = 0.0;
  ci.R[6] = 0.0;
  ci.R[7] = 0.0;
  ci.R[8] = 1.0;

  // set projection matrix
  ci.P[0] = ci.K[0];
  ci.P[1] = ci.K[1];
  ci.P[2] = ci.K[2];
  ci.P[3] = 0.0;
  ci.P[4] = ci.K[3];
  ci.P[5] = ci.K[4];
  ci.P[6] = ci.K[5];
  ci.P[7] = 0.0;
  ci.P[8] = ci.K[6];
  ci.P[9] = ci.K[7];
  ci.P[10] = ci.K[8];
  ci.P[11] = 0.0;

  return ci;
}

std::map<int, Eigen::Vector3f> WorldCoords::generateTagsInArmCoords(){
    std::map<int, Eigen::Vector3f> coords;
    coords.insert(std::make_pair(0, Eigen::Vector3f(0.2285,	0.0655, 0.)));
    coords.insert(std::make_pair(1, Eigen::Vector3f(0.2285,	0., 0.)));
    coords.insert(std::make_pair(2, Eigen::Vector3f(0.2285,	-0.0655, 0.)));
    coords.insert(std::make_pair(3, Eigen::Vector3f(0.1555,	0.0655, 0.)));
    coords.insert(std::make_pair(4, Eigen::Vector3f(0.1555,	0., 0.)));
    coords.insert(std::make_pair(5, Eigen::Vector3f(0.1555,	-0.0655, 0.)));
    coords.insert(std::make_pair(6, Eigen::Vector3f(0.0815,	0.0655, 0.)));
    coords.insert(std::make_pair(7, Eigen::Vector3f(0.0815,	0., 0.)));
    coords.insert(std::make_pair(8, Eigen::Vector3f(0.0815,	-0.0655, 0.)));
    return coords;
}

void WorldCoords::run(){
    rs2::pipeline_profile profile = pipe.start();
    float depth_scale = get_depth_scale(profile.get_device());
    rs2_stream align_to = find_stream_to_align(profile.get_streams());
    rs2::align align(align_to);
    // ros::Publisher depth_pub = n.advertise<std_msgs::Float32>("depth", 1000);
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 1);
    ros::Publisher info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);
    info_msg = generateCalibrationData();
    rs2_intr = createRs2Intrinsics();
    tag_centers_3d_arm = generateTagsInArmCoords();

    bool extrinsics_calculated = false;
    int seq = 0;
      while(ros::ok()){
        // std::cout<<"running!"<<std::endl;
        rs2::frameset frameset = pipe.wait_for_frames();
        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }

        //Get processed aligned frame
        auto processed = align.process(frameset);

        // Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }
        // Passing both frames to remove_background so it will "strip" the background
        // NOTE: in this example, we alter the buffer of the other frame, instead of copying it and altering the copy
        //       This behavior is not recommended in real application since the other frame could be used elsewhere
        // WorldCoords::remove_background(other_frame, aligned_depth_frame, depth_scale);//, depth_clipping_distance);
        const int w = other_frame.as<rs2::video_frame>().get_width();
        const int h = other_frame.as<rs2::video_frame>().get_height();
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)other_frame.get_data(), cv::Mat::AUTO_STEP);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), 
                                                       sensor_msgs::image_encodings::RGB8, 
                                                       image).toImageMsg();
        msg->header.stamp = ros::Time::now();
        image_pub.publish(msg);
        info_msg.header.stamp = msg->header.stamp;
        info_pub.publish(info_msg);
        if (!extrinsics_calculated){
            extrinsics_calculated = calculateExtrinsics(tag_centers_3d_cam, tag_centers_3d_arm, R, t);
        }
        ros::spinOnce();
    }
}

float WorldCoords::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

void WorldCoords::remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale)//, float clipping_dist)
{
    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();

    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];
            // std::cout<<"x: "<<x<<" y: "<<y<<" "<<pixels_distance<<std::endl;
            // Check if the depth value is invalid (<=0) or greater than the threashold
            // if (pixels_distance <= 0.f)// || pixels_distance > clipping_dist)
            // {
                // Calculate the offset in other frame's buffer to current pixel
                auto offset = depth_pixel_index * other_bpp;

                // Set pixel to "background" color (0x999999)
                // std::memset(&p_other_frame[offset], 0x99, other_bpp);
                std::memset(&p_other_frame[offset], pixels_distance * 100.0f, other_bpp);
            // }
        }
    }
}

rs2_stream WorldCoords::find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool WorldCoords::profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

void WorldCoords::processTagCentersClbk(const std_msgs::Float32MultiArray& msg){
    // std::cout<<"here"<<std::endl;
    if (msg.layout.dim[0].size == 27){
        for (int i = 0; i < msg.layout.dim[0].size; i = i+3){
            auto it = tag_centers_pix_cam.find(static_cast<int>(msg.data[i]));
            if (it != tag_centers_pix_cam.end()){
                it->second[0] = msg.data[i+1];
                it->second[1] = msg.data[i+2];
            } else {
                tag_centers_pix_cam.insert(std::make_pair<int, Eigen::Vector2f>(static_cast<int>(msg.data[i]), Eigen::Vector2f(msg.data[i+1], msg.data[i+2])));
            }
        }
    }

    // if (tag_centers_pix_cam.find(0) != tag_centers_pix_cam.end()){
    //     std::cout<<tag_centers_pix_cam.find(0)->second[0]<<" "<<tag_centers_pix_cam.find(0)->second[1]<<std::endl;
    // }
}

void WorldCoords::processTag3dCentersClbk(const lfm::AprilTagDetectionArray& msg){
    // std::cout<<sizeof(msg.detections)<<std::endl;
    // std::cout<<sizeof(msg.detections[0])<<std::endl;
    // for (int i = 0; i < sizeof(msg.detections)/sizeof(msg.detections[0]); i++){
    for (int i = 0; i < 9; i++){
        int id = msg.detections[i].id[0];
        std::cout<<"id: "<<id<<std::endl;
        auto it = tag_centers_3d_cam.find(id);
        Eigen::Vector3f pose = Eigen::Vector3f(msg.detections[i].pose.pose.pose.position.x,
                                               msg.detections[i].pose.pose.pose.position.y,
                                               msg.detections[i].pose.pose.pose.position.z);
        if (it != tag_centers_3d_cam.end()){
            it->second << pose;
        } else {
            // tag_centers_3d_cam.insert(std::make_pair<int, Eigen::Vector3f>(id, pose));
            tag_centers_3d_cam.insert(std::make_pair(id, pose));
        }
    }

    if (tag_centers_3d_cam.find(0) != tag_centers_3d_cam.end()){
        std::cout<<tag_centers_3d_cam.find(0)->second[0]<<" "<<tag_centers_3d_cam.find(0)->second[1]<<" "<<tag_centers_3d_cam.find(0)->second[2]<<std::endl;
    }
}

rs2_intrinsics WorldCoords::createRs2Intrinsics(){
    rs2_intrinsics intr;
    intr.width = info_msg.width;
    intr.height = info_msg.height;
    intr.coeffs[0] = info_msg.D[0];
    intr.coeffs[1] = info_msg.D[1];
    intr.coeffs[2] = info_msg.D[2];
    intr.coeffs[3] = info_msg.D[3];
    intr.coeffs[4] = info_msg.D[4];
    intr.ppx = info_msg.K[2];
    intr.ppx = info_msg.K[5];
    intr.fx = info_msg.K[0];
    intr.fy = info_msg.K[4];
    intr.model = rs2_distortion::RS2_DISTORTION_INVERSE_BROWN_CONRADY;
    return intr;
}

bool WorldCoords::calculateExtrinsics(const std::map<int, Eigen::Vector3f>& cam, 
                                      const std::map<int, Eigen::Vector3f>& arm, 
                                      Eigen::Matrix3f& R, 
                                      Eigen::Vector3f& t){
    int num_points = cam.size();
    if (num_points == 9){
        Eigen::MatrixXf X(3, cam.size());
        Eigen::MatrixXf Y(3, cam.size());
        Eigen::Vector3f pbar, qbar;
        pbar << 0.0, 0.0, 0.0;
        qbar << 0.0, 0.0, 0.0;
        for (int i = 0; i < num_points; i++){
            auto it_cam = cam.find(i);
            auto it_arm = arm.find(i);
            // assume wi = 1.0
            pbar += it_cam->second;
            qbar += it_cam->second;
        }

        // calc weighted centroids
        pbar /= num_points;
        qbar /= num_points;
        
        // calc centered vectors
        for (int i = 0; i < num_points; i++){
            auto it_cam = cam.find(i);
            auto it_arm = arm.find(i);
            X.col(i) << it_cam->second - pbar;
            Y.col(i) << it_arm->second - qbar;
        }

        // calc covariance matrix
        Eigen::Matrix3f S(3, 3);
        S = X * Y.transpose();

        // calc SVD
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // calc R
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();
        float det = (V*U.transpose()).determinant();
        Eigen::Vector3f vec;
        vec << 1, 1, det; 
        Eigen::Matrix3f mid = vec.asDiagonal();
        R = V * mid * U.transpose();

        // calc t
        t = qbar - R * pbar;

        return true;
    } else return false;

}

// void WorldCoords::calcRt(){

// }

// void WorldCoords::segmentByColor(cv::Mat & image, cv::Scalar target_color){
//     cv::Scalar upper, lower;
//     int range = 20;
//     for (int i = 0; i<3; i++){
//         // std::cout<<target_color[i]<<std::endl;
//         upper[i] = std::min<int>(target_color[i]+range, 255);
//         lower[i] = std::max<int>(target_color[i]-range, 0.0f);
//         // std::cout<<upper[i]<<std::endl;
//     }
//     cv::inRange(image, lower, upper, image);
// }

int main (int argc, char **argv)
{
	ros::init(argc, argv, "lfm");
    WorldCoords wc;
    wc.run();
	return 0;
}



// // // void render_slider(rect location, float& clipping_dist);
// // void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
// // float get_depth_scale(rs2::device dev);
// // rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
// // bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

// int main(int argc, char * argv[]) try
// {
//     // Create and initialize GUI related objects
//     // window app(1280, 720, "CPP - Align Example"); // Simple window handling
//     // ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
//     rs2::colorizer c;                          // Helper to colorize depth images
//     // texture renderer;                     // Helper for renderig images

//     // Create a pipeline to easily configure and start the camera
//     rs2::pipeline pipe;
//     //Calling pipeline's start() without any additional parameters will start the first device
//     // with its default streams.
//     //The start function returns the pipeline profile which the pipeline used to start the device
//     rs2::pipeline_profile profile = pipe.start();

//     // Each depth camera might have different units for depth pixels, so we get it here
//     // Using the pipeline's profile, we can retrieve the device that the pipeline uses
//     float depth_scale = get_depth_scale(profile.get_device());

//     //Pipeline could choose a device that does not have a color stream
//     //If there is no color stream, choose to align depth to another stream
//     rs2_stream align_to = find_stream_to_align(profile.get_streams());

//     // Create a rs2::align object.
//     // rs2::align allows us to perform alignment of depth frames to others frames
//     //The "align_to" is the stream type to which we plan to align depth frames.
//     rs2::align align(align_to);

//     // Define a variable for controlling the distance to clip
//     // float depth_clipping_distance = 1.f;

//     while (app) // Application still alive?
//     {
//         // Using the align object, we block the application until a frameset is available
//         rs2::frameset frameset = pipe.wait_for_frames();

//         // rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
//         // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
//         //  after the call to wait_for_frames();
//         if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
//         {
//             //If the profile was changed, update the align object, and also get the new device's depth scale
//             profile = pipe.get_active_profile();
//             align_to = find_stream_to_align(profile.get_streams());
//             align = rs2::align(align_to);
//             depth_scale = get_depth_scale(profile.get_device());
//         }

//         //Get processed aligned frame
//         auto processed = align.process(frameset);

//         // Trying to get both other and aligned depth frames
//         rs2::video_frame other_frame = processed.first(align_to);
//         rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

//         //If one of them is unavailable, continue iteration
//         if (!aligned_depth_frame || !other_frame)
//         {
//             continue;
//         }
//         // Passing both frames to remove_background so it will "strip" the background
//         // NOTE: in this example, we alter the buffer of the other frame, instead of copying it and altering the copy
//         //       This behavior is not recommended in real application since the other frame could be used elsewhere
//         remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance);

//         // Taking dimensions of the window for rendering purposes
//         float w = static_cast<float>(app.width());
//         float h = static_cast<float>(app.height());

//         // At this point, "other_frame" is an altered frame, stripped form its background
//         // Calculating the position to place the frame in the window
//         rect altered_other_frame_rect{ 0, 0, w, h };
//         altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.get_width()),static_cast<float>(other_frame.get_height()) });

//         // Render aligned image
//         renderer.render(other_frame, altered_other_frame_rect);

//         // // The example also renders the depth frame, as a picture-in-picture
//         // // Calculating the position to place the depth frame in the window
//         // rect pip_stream{ 0, 0, w / 5, h / 5 };
//         // pip_stream = pip_stream.adjust_ratio({ static_cast<float>(aligned_depth_frame.get_width()),static_cast<float>(aligned_depth_frame.get_height()) });
//         // pip_stream.x = altered_other_frame_rect.x + altered_other_frame_rect.w - pip_stream.w - (std::max(w, h) / 25);
//         // pip_stream.y = altered_other_frame_rect.y + altered_other_frame_rect.h - pip_stream.h - (std::max(w, h) / 25);

//         // // Render depth (as picture in pipcture)
//         // renderer.upload(c.process(aligned_depth_frame));
//         // renderer.show(pip_stream);

//         // // Using ImGui library to provide a slide controller to select the depth clipping distance
//         // ImGui_ImplGlfw_NewFrame(1);
//         // render_slider({ 5.f, 0, w, h }, depth_clipping_distance);
//         // ImGui::Render();
//     }
//     return EXIT_SUCCESS;
// }
// catch (const rs2::error & e)
// {
//     std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//     return EXIT_FAILURE;
// }
// catch (const std::exception & e)
// {
//     std::cerr << e.what() << std::endl;
//     return EXIT_FAILURE;
// }

// float get_depth_scale(rs2::device dev)
// {
//     // Go over the device's sensors
//     for (rs2::sensor& sensor : dev.query_sensors())
//     {
//         // Check if the sensor if a depth sensor
//         if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
//         {
//             return dpt.get_depth_scale();
//         }
//     }
//     throw std::runtime_error("Device does not have a depth sensor");
// }

// void render_slider(rect location, float& clipping_dist)
// {
//     // Some trickery to display the control nicely
//     static const int flags = ImGuiWindowFlags_NoCollapse
//         | ImGuiWindowFlags_NoScrollbar
//         | ImGuiWindowFlags_NoSavedSettings
//         | ImGuiWindowFlags_NoTitleBar
//         | ImGuiWindowFlags_NoResize
//         | ImGuiWindowFlags_NoMove;
//     const int pixels_to_buttom_of_stream_text = 25;
//     const float slider_window_width = 30;

//     ImGui::SetNextWindowPos({ location.x, location.y + pixels_to_buttom_of_stream_text });
//     ImGui::SetNextWindowSize({ slider_window_width + 20, location.h - (pixels_to_buttom_of_stream_text * 2) });

//     //Render the vertical slider
//     ImGui::Begin("slider", nullptr, flags);
//     ImGui::PushStyleColor(ImGuiCol_FrameBg, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
//     ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
//     ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
//     auto slider_size = ImVec2(slider_window_width / 2, location.h - (pixels_to_buttom_of_stream_text * 2) - 20);
//     ImGui::VSliderFloat("", slider_size, &clipping_dist, 0.0f, 6.0f, "", 1.0f, true);
//     if (ImGui::IsItemHovered())
//         ImGui::SetTooltip("Depth Clipping Distance: %.3f", clipping_dist);
//     ImGui::PopStyleColor(3);

//     //Display bars next to slider
//     float bars_dist = (slider_size.y / 6.0f);
//     for (int i = 0; i <= 6; i++)
//     {
//         ImGui::SetCursorPos({ slider_size.x, i * bars_dist });
//         std::string bar_text = "- " + std::to_string(6-i) + "m";
//         ImGui::Text("%s", bar_text.c_str());
//     }
//     ImGui::End();
// }

// void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist)
// {
//     const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
//     uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

//     int width = other_frame.get_width();
//     int height = other_frame.get_height();
//     int other_bpp = other_frame.get_bytes_per_pixel();

//     #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
//     for (int y = 0; y < height; y++)
//     {
//         auto depth_pixel_index = y * width;
//         for (int x = 0; x < width; x++, ++depth_pixel_index)
//         {
//             // Get the depth value of the current pixel
//             auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

//             // Check if the depth value is invalid (<=0) or greater than the threashold
//             if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
//             {
//                 // Calculate the offset in other frame's buffer to current pixel
//                 auto offset = depth_pixel_index * other_bpp;

//                 // Set pixel to "background" color (0x999999)
//                 std::memset(&p_other_frame[offset], 0x99, other_bpp);
//             }
//         }
//     }
// }

// rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
// {
//     //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
//     //We prioritize color streams to make the view look better.
//     //If color is not available, we take another stream that (other than depth)
//     rs2_stream align_to = RS2_STREAM_ANY;
//     bool depth_stream_found = false;
//     bool color_stream_found = false;
//     for (rs2::stream_profile sp : streams)
//     {
//         rs2_stream profile_stream = sp.stream_type();
//         if (profile_stream != RS2_STREAM_DEPTH)
//         {
//             if (!color_stream_found)         //Prefer color
//                 align_to = profile_stream;

//             if (profile_stream == RS2_STREAM_COLOR)
//             {
//                 color_stream_found = true;
//             }
//         }
//         else
//         {
//             depth_stream_found = true;
//         }
//     }

//     if(!depth_stream_found)
//         throw std::runtime_error("No Depth stream available");

//     if (align_to == RS2_STREAM_ANY)
//         throw std::runtime_error("No stream found to align with Depth");

//     return align_to;
// }

// bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
// {
//     for (auto&& sp : prev)
//     {
//         //If previous profile is in current (maybe just added another)
//         auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
//         if (itr == std::end(current)) //If it previous stream wasn't found in current
//         {
//             return true;
//         }
//     }
//     return false;
// }

// {
//   float to_pixel[2];
//   rs2::depth_frame depth_frame;
//   float depth_scale = get_depth_scale(profile.get_device());
//   rs2_project_color_pixel_to_depth_pixel(to_pixel, depth_frame, )
// }
