#include "lfm.h"

class Controller {
    public:
        void run();
        // void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale);//, float clipping_dist);
        float get_depth_scale(rs2::device dev);
        rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
        bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
        std::map<int, Eigen::Vector3f> generateTagsInArmCoords();
        // void processTagCentersClbk(const std_msgs::Float32MultiArray& msg);
        void processTag3dCentersClbk(const lfm::AprilTagDetectionArray& msg);
        void processActionClbk(const lfm::Action& msg);
        Eigen::Vector3f getTagCoordsMillimeters(const int& tag_id);
        Eigen::Vector3f calcReleaseCoords(const int& tag_id, const float& dist, const float& angle);
        Eigen::Vector3f cartesianToPolar(const Eigen::Vector3f& pos);
        void getExtrinsicParams();
        void pickTagClbk(const std_msgs::Int32& msg);
        void statusClbk(const swiftpro::SwiftproState& msg);
        void sendPumpCmd();
        bool checkReached();
        void moveVertical();
        void updateState();
        void sendPosCmd();
        bool inBounds(const Eigen::Vector3f& pos);
        Eigen::Vector3f transformCamToArm(const Eigen::Vector3f& cam);
        rs2_intrinsics createRs2Intrinsics();
        sensor_msgs::CameraInfo generateCalibrationData();
        bool calculateExtrinsics(const std::map<int, Eigen::Vector3f>& cam, 
                                 const std::map<int, Eigen::Vector3f>& arm, 
                                 Eigen::Matrix3f& R, 
                                 Eigen::Vector3f& t);

        ros::NodeHandle n;
        rs2::pipeline pipe;
        // ros::Subscriber tag_centers_sub = n.subscribe("/apriltags2_ros_continuous_node/tag_pixel_centers", 1000, &Controller::processTagCentersClbk, this);
        ros::Subscriber tag_centers_3d_sub = n.subscribe("/tag_detections", 1000, &Controller::processTag3dCentersClbk, this);
        // ros::Subscriber pick_tag_sub = n.subscribe("/pick_tag", 1, &Controller::pickTagClbk, this);
	    ros::Subscriber status_sub = n.subscribe("/SwiftproState_topic", 1, &Controller::statusClbk, this);
	    ros::Subscriber action_sub = n.subscribe("/action_request", 1, &Controller::processActionClbk, this);

        ros::Publisher arm_pos_cmd_pub = n.advertise<swiftpro::position>("/position_write_topic", 1);
        ros::Publisher ready_for_action_pub = n.advertise<std_msgs::Bool>("/ready_for_action", 1);
        ros::Publisher pump_pub = n.advertise<swiftpro::status>("/pump_topic", 1);
        // ros::Publisher tag_centers_arm_pub = n.advertise<geometry_msgs::PoseStamped>("/tag_centers_arm", 1);
        // std::map<int, Eigen::Vector2f> tag_centers_pix_cam;
        std::map<int, Eigen::Vector3f> tag_centers_3d_cam;
        std::map<int, Eigen::Vector3f> tag_centers_3d_arm;
        sensor_msgs::CameraInfo info_msg;
        rs2_intrinsics rs2_intr;
        Eigen::Matrix3f R;
        Eigen::Vector3f t;
        int tag_to_pick;
        bool pump_active;
        Eigen::Vector3f arm_pos_desired;
        Eigen::Vector3f arm_pos_actual;
        std::vector<Eigen::Vector3f> arm_pos_sequence;

        float pos_error_tolerance = 1.0; // mm
        float standoff_height = 100.0; // mm 40.0
        // float clear_height = 200.0; // mm
        float pick_height = 70.0; // mm 1.0

        Eigen::Vector3f home_pos = Eigen::Vector3f(10.0, -150.0, 100.0);

        std::vector<float> R_limits {100.0, 260.0};
        std::vector<float> th_limits {-45.0, 45.0};
        std::vector<float> z_limits {-1.0, 200.0};
        
        enum ArmState{
            IDLE = -1,
            HOVER_START,
            PICK,
            RELEASE,
            HOVER_END,
            END_OF_SEQ
        };

        int arm_state = ArmState::IDLE;
};

sensor_msgs::CameraInfo Controller::generateCalibrationData()
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

// std::map<int, Eigen::Vector3f> Controller::generateTagsInArmCoords(){
//     std::map<int, Eigen::Vector3f> coords;
//     coords.insert(std::make_pair(0, Eigen::Vector3f(0.2435,	-0.0655, 0.)));
//     coords.insert(std::make_pair(1, Eigen::Vector3f(0.2435,	0., 0.)));
//     coords.insert(std::make_pair(2, Eigen::Vector3f(0.2435,	0.0645, 0.)));
//     coords.insert(std::make_pair(3, Eigen::Vector3f(0.1705,	-0.0655, 0.)));
//     coords.insert(std::make_pair(4, Eigen::Vector3f(0.1705,	0., 0.)));
//     coords.insert(std::make_pair(5, Eigen::Vector3f(0.1705,	0.0645, 0.)));
//     coords.insert(std::make_pair(6, Eigen::Vector3f(0.0965,	-0.0655, 0.)));
//     coords.insert(std::make_pair(7, Eigen::Vector3f(0.0965,	0., 0.)));
//     coords.insert(std::make_pair(8, Eigen::Vector3f(0.0965,	0.0645, 0.)));
//     return coords;
// }

void Controller::run(){
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
    ros::Rate loop_rate = 4;
    // tag_centers_3d_arm = generateTagsInArmCoords();

    // bool extrinsics_calculated = false;
    // ROS_INFO("Are all tags out of the scene?");

    getExtrinsicParams();

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
        // Controller::remove_background(other_frame, aligned_depth_frame, depth_scale);//, depth_clipping_distance);
        // const int w = other_frame.as<rs2::video_frame>().get_width();
        // const int h = other_frame.as<rs2::video_frame>().get_height();
        // cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)other_frame.get_data(), cv::Mat::AUTO_STEP);
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
        // if (!extrinsics_calculated){
        //     swiftpro::position home;
        //     home.x = home_pos[0];
        //     home.y = home_pos[1];
        //     home.z = home_pos[2];
        //     arm_pos_desired = home_pos;//Eigen::Vector3f(home.x, home.y, home.z);
        //     arm_pos_cmd_pub.publish(home); // move it out of the way
        //     if (Controller::checkReached()){
        //         ROS_INFO("Calculating extrinsics...");
        //         extrinsics_calculated = calculateExtrinsics(tag_centers_3d_cam, tag_centers_3d_arm, R, t);
        //     }
        // }

        // do the real work
        updateState();
        sendPosCmd();
        sendPumpCmd();
        std::cout<<"z desired: "<<arm_pos_desired[2]<<std::endl;;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// void Controller::processTagCentersClbk(const std_msgs::Float32MultiArray& msg){
//     // std::cout<<"here"<<std::endl;
//     if (msg.layout.dim[0].size == 27){
//         for (int i = 0; i < msg.layout.dim[0].size; i = i+3){
//             auto it = tag_centers_pix_cam.find(static_cast<int>(msg.data[i]));
//             if (it != tag_centers_pix_cam.end()){
//                 it->second[0] = msg.data[i+1];
//                 it->second[1] = msg.data[i+2];
//             } else {
//                 tag_centers_pix_cam.insert(std::make_pair<int, Eigen::Vector2f>(static_cast<int>(msg.data[i]), Eigen::Vector2f(msg.data[i+1], msg.data[i+2])));
//             }
//         }
//     }
// }

void Controller::getExtrinsicParams(){
    std::vector<float> R_param, t_param;
    n.getParam("R", R_param);
    n.getParam("t", t_param);
    assert(R_param.size() == 9 && t_param.size() == 3);
    
    R(0,0) = R_param[0];
    R(0,1) = R_param[1];
    R(0,2) = R_param[2];
    R(1,0) = R_param[3];
    R(1,1) = R_param[4];
    R(1,2) = R_param[5];
    R(2,0) = R_param[6];
    R(2,1) = R_param[7];
    R(2,2) = R_param[8];

    t(0) = t_param[0];
    t(1) = t_param[1];
    t(2) = t_param[2];
}

void Controller::processTag3dCentersClbk(const lfm::AprilTagDetectionArray& msg){
    int num_detections = msg.detections.size();
    for (int i = 0; i < num_detections; i++){
        int id = msg.detections[i].id[0];
        // std::cout<<"id: "<<id<<std::endl;
        auto it = tag_centers_3d_cam.find(id);
        Eigen::Vector3f pose = Eigen::Vector3f(msg.detections[i].pose.pose.pose.position.x,
                                               msg.detections[i].pose.pose.pose.position.y,
                                               msg.detections[i].pose.pose.pose.position.z);
        if (it != tag_centers_3d_cam.end()){
            it->second << pose;
        } else {
            tag_centers_3d_cam.insert(std::make_pair(id, pose));
        }
    }
}

// void Controller::pickTagClbk(const std_msgs::Int32& msg){
//     int tag_id = msg.data;
//     std::cout<<"tag_id: "<<tag_id<<std::endl;
//     if (tag_centers_3d_cam.find(tag_id) != tag_centers_3d_cam.end()){
//         arm_pos_desired = Controller::transformCamToArm(tag_centers_3d_cam.find(tag_id)->second); // look up tag location
//         // arm_pos_desired = tag_centers_3d_cam.find(tag_id)->second; // look up tag location
//         arm_pos_desired[2] = 10.0f; // mm
//         swiftpro::position pos;
//         pos.x = arm_pos_desired[0] * 1000.0f; // mm
//         pos.y = arm_pos_desired[1] * 1000.0f; // mm
//         pos.z = arm_pos_desired[2];
//         arm_pos_cmd_pub.publish(pos); // send to robot
//     }
// }

void Controller::updateState(){
    switch(arm_state){
        case ArmState::IDLE:
            arm_pos_desired = home_pos;
            break;
        case ArmState::END_OF_SEQ:{
            arm_state == ArmState::IDLE;
            arm_pos_sequence.clear();
            // std_msgs::Bool msg;
            // msg.data = true;
            // ready_for_action_pub.publish(msg);
            break;}
        default:
            if (Controller::checkReached()){
                arm_state++;
                arm_pos_desired = arm_pos_sequence[arm_state];
                std::cout<<"arm state: "<<arm_state<<std::endl;
                // std::cout<<arm_pos_desired<<std::endl;
                ros::Duration(1.5).sleep();
            }
            break;
    }
}

bool Controller::inBounds(const Eigen::Vector3f& pos){
    bool in_bounds;
    Eigen::Vector3f polar = Controller::cartesianToPolar(pos);
    if (polar[0] <= R_limits[0]  || polar[0] >= R_limits[1]  || 
        polar[1] <= th_limits[0] || polar[1] >= th_limits[1] || 
        polar[2] <= z_limits[0]  || polar[2] >= z_limits[1]) {
            in_bounds = false;
            std::cout<<"Requested point out of bounds!"<<std::endl;
    } else {
        in_bounds = true;
    } 
    return in_bounds;
}

void Controller::sendPosCmd(){
    swiftpro::position pos;
    pos.x = arm_pos_desired[0];
    pos.y = arm_pos_desired[1];
    pos.z = arm_pos_desired[2];
    arm_pos_cmd_pub.publish(pos); // send to robot    
}

void Controller::sendPumpCmd(){
    swiftpro::status msg;
    if (arm_state == ArmState::PICK){
        msg.status = 1;
    } else msg.status = 0;
    pump_pub.publish(msg);
}

void Controller::statusClbk(const swiftpro::SwiftproState& msg){
    arm_pos_actual[0] = msg.x;
    arm_pos_actual[1] = msg.y;
    arm_pos_actual[2] = msg.z;
}

bool Controller::checkReached(){
    for (int i = 0; i < 3; i++){
        if (abs(arm_pos_desired[i] - arm_pos_actual[i]) > pos_error_tolerance){
            std::cout<<"still travelling..."<<std::endl;
            return false;
        }
    }
    return true;
}

// void Controller::moveVertical(){
//     if (!is_down){
//         arm_pos_desired[2] = 0.0f;
//     } else {
//         arm_pos_desired[2] = 0.0f;
//     }
//     swiftpro::position pos;
//     pos.x = arm_pos_desired[0];
//     pos.y = arm_pos_desired[1];
//     pos.z = arm_pos_desired[2];
//     arm_pos_cmd_pub.publish(pos); // send to robot
// }

Eigen::Vector3f Controller::transformCamToArm(const Eigen::Vector3f& cam){
    return R * cam + t; 
}

void Controller::processActionClbk(const lfm::Action& msg){
    Eigen::Vector3f hover_start, pick, release, hover_end, clear; 
    int tag_id = msg.target_tag;
    Eigen::Vector3f coords = Controller::getTagCoordsMillimeters(tag_id);
    hover_start[0] = coords[0];
    hover_start[1] = coords[1];
    hover_start[2] = standoff_height;
    pick = hover_start;
    pick[2] = pick_height;
    release = Controller::calcReleaseCoords(tag_id, msg.dist, msg.angle); // msg.dist must be in mm!
    release[2] = pick_height;
    hover_end = release;
    hover_end[2] = standoff_height;
    clear = home_pos;
    arm_pos_sequence = {hover_start, pick, release, hover_end, clear};
    for (auto pos : arm_pos_sequence){
        std::cout<<pos<<std::endl;
        if (!Controller::inBounds(pos)){
            std::cout<<"Illegal move requested. Try again!"<<std::endl;
            arm_state = ArmState::IDLE;
            arm_pos_sequence.clear();
            break;
        }
    }
    arm_state = ArmState::HOVER_START; // if all clear, move to the first position!
    arm_pos_desired = arm_pos_sequence[arm_state];
    // tag_centers_arm_pub.publish(/*TODO*/);
}

Eigen::Vector3f Controller::getTagCoordsMillimeters(const int& tag_id){
    Eigen::Vector3f coords;
    if (tag_centers_3d_cam.find(tag_id) != tag_centers_3d_cam.end()){
        coords = Controller::transformCamToArm(tag_centers_3d_cam.find(tag_id)->second);
    }
    coords *= 1000.0; // this converts to mm!
    return coords;
}

Eigen::Vector3f Controller::calcReleaseCoords(const int& tag_id, const float& dist, const float& angle){
    Eigen::Vector3f start, end; 
    start = Controller::getTagCoordsMillimeters(tag_id);
    float dx = dist * cos(angle * M_PI / 180.0);
    float dy = dist * sin(angle * M_PI / 180.0);
    end[0] = start[0] + dx;
    end[1] = start[1] + dy;
    end[2] = start[2];
    std::cout<<"Start: "<<start<<std::endl;
    std::cout<<"End: "<<end<<std::endl;
    return end;
}

Eigen::Vector3f Controller::cartesianToPolar(const Eigen::Vector3f& pos){
    Eigen::Vector3f polar; // {R, th, z}
    polar[0] = sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
    polar[1] = atan2(pos[1], pos[0]);
    polar[2] = pos[2];
    return polar;
}

rs2_intrinsics Controller::createRs2Intrinsics(){
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

// bool Controller::calculateExtrinsics(const std::map<int, Eigen::Vector3f>& cam, 
//                                       const std::map<int, Eigen::Vector3f>& arm, 
//                                       Eigen::Matrix3f& R, 
//                                       Eigen::Vector3f& t){
//     int num_points = cam.size();
//     if (num_points == 9){
//         Eigen::MatrixXf X(3, cam.size());
//         Eigen::MatrixXf Y(3, cam.size());
//         Eigen::Vector3f pbar, qbar;
//         pbar << 0.0, 0.0, 0.0;
//         qbar << 0.0, 0.0, 0.0;
//         for (int i = 0; i < num_points; i++){
//             auto it_cam = cam.find(i);
//             auto it_arm = arm.find(i);
//             // assume wi = 1.0
//             pbar += it_cam->second;
//             qbar += it_arm->second;
//         }

//         // calc weighted centroids
//         pbar /= num_points;
//         qbar /= num_points;
        
//         // std::cout<<pbar<<std::endl;        
//         // std::cout<<qbar<<std::endl;        

//         // calc centered vectors
//         for (int i = 0; i < num_points; i++){
//             auto it_cam = cam.find(i);
//             auto it_arm = arm.find(i);
//             X.col(i) << it_cam->second - pbar;
//             Y.col(i) << it_arm->second - qbar;
//         }
//         // std::cout<<"X: \n";
//         // std::cout<<X<<std::endl;
//         // std::cout<<"Y: \n";
//         // std::cout<<Y<<std::endl;
//         // calc covariance matrix
//         Eigen::Matrix3f S(3, 3);
//         S = X * Y.transpose();

//         // calc SVD
//         Eigen::JacobiSVD<Eigen::Matrix3f> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);

//         // calc R
//         Eigen::Matrix3f U = svd.matrixU();
//         Eigen::Matrix3f V = svd.matrixV();
//         float det = (V*U.transpose()).determinant();
//         Eigen::Vector3f vec;
//         vec << 1, 1, det; 
//         Eigen::Matrix3f mid = vec.asDiagonal();
//         R = V * mid * U.transpose();

//         // calc t
//         t = qbar - R * pbar;

//         // std::cout<<"R:"<<R<<std::endl;
//         // std::cout<<"t:"<<t<<std::endl;

//         std::vector<float> R_param;
//         R_param.push_back(R(0,0));
//         R_param.push_back(R(0,1));
//         R_param.push_back(R(0,2));
//         R_param.push_back(R(1,0));
//         R_param.push_back(R(1,1));
//         R_param.push_back(R(1,2));
//         R_param.push_back(R(2,0));
//         R_param.push_back(R(2,1));
//         R_param.push_back(R(2,2));

//         std::vector<float> t_param;
//         t_param.push_back(t(0));
//         t_param.push_back(t(1));
//         t_param.push_back(t(2)); 

//         n.setParam("R", R_param);
//         n.setParam("t", t_param);

//         ROS_INFO("Extrinsics calculated!");

//         return true;
//     } else return false;

// }

float Controller::get_depth_scale(rs2::device dev)
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

// void Controller::remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale)//, float clipping_dist)
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
//             // std::cout<<"x: "<<x<<" y: "<<y<<" "<<pixels_distance<<std::endl;
//             // Check if the depth value is invalid (<=0) or greater than the threashold
//             // if (pixels_distance <= 0.f)// || pixels_distance > clipping_dist)
//             // {
//                 // Calculate the offset in other frame's buffer to current pixel
//                 auto offset = depth_pixel_index * other_bpp;

//                 // Set pixel to "background" color (0x999999)
//                 // std::memset(&p_other_frame[offset], 0x99, other_bpp);
//                 std::memset(&p_other_frame[offset], pixels_distance * 100.0f, other_bpp);
//             // }
//         }
//     }
// }

rs2_stream Controller::find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
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

bool Controller::profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
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

int main (int argc, char **argv)
{
	ros::init(argc, argv, "lfm");
    Controller c;
    c.run();
	return 0;
}