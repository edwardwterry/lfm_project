
#include "lfm.h"

class WorldCoords {
    public:
        void run();
        void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale);//, float clipping_dist);
        float get_depth_scale(rs2::device dev);
        rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
        bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
        std::map<int, Eigen::Vector3f> generateTagsInArmCoords();
        void processTagCentersClbk(const std_msgs::Float32MultiArray& msg);
        void processTag3dCentersClbk(const lfm::AprilTagDetectionArray& msg);
        void processActionClbk(const lfm::Action& msg);
        Eigen::Vector3f getTagCoords(const int& tag_id);
        Eigen::Vector3f calcReleaseCoords(const int& tag_id, const float& dist, const float& angle);
        Eigen::Vector3f cartesianToPolar(const Eigen::Vector3f& pos);
        void pickTagClbk(const std_msgs::Int32& msg);
        void statusClbk(const swiftpro::SwiftproState& msg);
        void managePump();
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
        // ros::Subscriber tag_centers_sub = n.subscribe("/apriltags2_ros_continuous_node/tag_pixel_centers", 1000, &WorldCoords::processTagCentersClbk, this);
        ros::Subscriber tag_centers_3d_sub = n.subscribe("/tag_detections", 1000, &WorldCoords::processTag3dCentersClbk, this);
        // ros::Subscriber pick_tag_sub = n.subscribe("/pick_tag", 1, &WorldCoords::pickTagClbk, this);
	    ros::Subscriber status_sub = n.subscribe("SwiftproState_topic", 1, &WorldCoords::statusClbk, this);

        ros::Publisher arm_pos_cmd_pub = n.advertise<swiftpro::position>("/arm_pos_cmd", 1);
        ros::Publisher pump_pub = n.advertise<swiftpro::position>("/pump_topic", 1); // TODO, check topic name
        std::map<int, Eigen::Vector2f> tag_centers_pix_cam;
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
        float standoff_height = 10.0; // mm
        float clear_height = 150.0; // mm
        float pick_height = 0.0; // mm

        Eigen::Vector3f home_pos = Eigen::Vector3f(10.0, 150.0, 100.0);

        std::vector<float> R_limits {80.0, 200.0};
        std::vector<float> th_limits {-90.0, 90.0};
        std::vector<float> z_limits {-1.0, 150.0};
        
        enum ArmState{
            IDLE = -1,
            HOVER_START,
            DROP,
            PICK,
            RELEASE,
            HOVER_END,
            END_OF_SEQ
        };

        int arm_state = ArmState::IDLE;
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
    coords.insert(std::make_pair(0, Eigen::Vector3f(0.2435,	-0.0655, 0.)));
    coords.insert(std::make_pair(1, Eigen::Vector3f(0.2435,	0., 0.)));
    coords.insert(std::make_pair(2, Eigen::Vector3f(0.2435,	0.0645, 0.)));
    coords.insert(std::make_pair(3, Eigen::Vector3f(0.1705,	-0.0655, 0.)));
    coords.insert(std::make_pair(4, Eigen::Vector3f(0.1705,	0., 0.)));
    coords.insert(std::make_pair(5, Eigen::Vector3f(0.1705,	0.0645, 0.)));
    coords.insert(std::make_pair(6, Eigen::Vector3f(0.0965,	-0.0655, 0.)));
    coords.insert(std::make_pair(7, Eigen::Vector3f(0.0965,	0., 0.)));
    coords.insert(std::make_pair(8, Eigen::Vector3f(0.0965,	0.0645, 0.)));
    return coords;
}

void WorldCoords::run(){
    rs2::pipeline_profile profile = pipe.start();
    // float depth_scale = get_depth_scale(profile.get_device());
    // rs2_stream align_to = find_stream_to_align(profile.get_streams());
    // rs2::align align(align_to);
    // ros::Publisher depth_pub = n.advertise<std_msgs::Float32>("depth", 1000);
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub = it.advertise("/camera/image_raw", 1);
    ros::Publisher info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);
    info_msg = generateCalibrationData();
    rs2_intr = createRs2Intrinsics();
    tag_centers_3d_arm = generateTagsInArmCoords();

    bool extrinsics_calculated = false;

    while(ros::ok()){
        // std::cout<<"running!"<<std::endl;
        rs2::frameset frameset = pipe.wait_for_frames();
        // if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        // {
        //     //If the profile was changed, update the align object, and also get the new device's depth scale
        //     profile = pipe.get_active_profile();
        //     align_to = find_stream_to_align(profile.get_streams());
        //     align = rs2::align(align_to);
        //     depth_scale = get_depth_scale(profile.get_device());
        // }

        // //Get processed aligned frame
        // auto processed = align.process(frameset);

        // // Trying to get both other and aligned depth frames
        // rs2::video_frame other_frame = processed.first(align_to);
        // rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        // //If one of them is unavailable, continue iteration
        // if (!aligned_depth_frame || !other_frame)
        // {
        //     continue;
        // }
        // Passing both frames to remove_background so it will "strip" the background
        // NOTE: in this example, we alter the buffer of the other frame, instead of copying it and altering the copy
        //       This behavior is not recommended in real application since the other frame could be used elsewhere
        // WorldCoords::remove_background(other_frame, aligned_depth_frame, depth_scale);//, depth_clipping_distance);
        // const int w = other_frame.as<rs2::video_frame>().get_width();
        // const int h = other_frame.as<rs2::video_frame>().get_height();
        // cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)other_frame.get_data(), cv::Mat::AUTO_STEP);
        const int w = frameset.as<rs2::video_frame>().get_width();
        const int h = frameset.as<rs2::video_frame>().get_height();
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)frameset.get_data(), cv::Mat::AUTO_STEP);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), 
                                                       sensor_msgs::image_encodings::RGB8, 
                                                       image).toImageMsg();
        msg->header.stamp = ros::Time::now();
        image_pub.publish(msg);
        info_msg.header.stamp = msg->header.stamp;
        info_pub.publish(info_msg);
        if (!extrinsics_calculated){
            swiftpro::position home;
            home.x = 10.0f;
            home.y = 150.0f;
            home.z = 100.0f;
            arm_pos_desired = Eigen::Vector3f(home.x, home.y, home.z);
            arm_pos_cmd_pub.publish(home); // move it out of the way
            if (WorldCoords::checkReached()){
                ROS_INFO("Calculating extrinsics...");
                extrinsics_calculated = calculateExtrinsics(tag_centers_3d_cam, tag_centers_3d_arm, R, t);
            }
        }
        updateState();
        sendPosCmd();
        managePump();
        ros::spinOnce();
    }
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
}

void WorldCoords::processTag3dCentersClbk(const lfm::AprilTagDetectionArray& msg){
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

// void WorldCoords::pickTagClbk(const std_msgs::Int32& msg){
//     int tag_id = msg.data;
//     std::cout<<"tag_id: "<<tag_id<<std::endl;
//     if (tag_centers_3d_cam.find(tag_id) != tag_centers_3d_cam.end()){
//         arm_pos_desired = WorldCoords::transformCamToArm(tag_centers_3d_cam.find(tag_id)->second); // look up tag location
//         // arm_pos_desired = tag_centers_3d_cam.find(tag_id)->second; // look up tag location
//         arm_pos_desired[2] = 10.0f; // mm
//         swiftpro::position pos;
//         pos.x = arm_pos_desired[0] * 1000.0f; // mm
//         pos.y = arm_pos_desired[1] * 1000.0f; // mm
//         pos.z = arm_pos_desired[2];
//         arm_pos_cmd_pub.publish(pos); // send to robot
//     }
// }

void WorldCoords::updateState(){
    switch(arm_state){
        case ArmState::IDLE:
            arm_pos_desired = home_pos;
            break;
        case ArmState::END_OF_SEQ:
            arm_state == ArmState::IDLE;
            arm_pos_sequence.clear();
            break;
        default:
            if (WorldCoords::checkReached()){
                arm_pos_desired = arm_pos_sequence[arm_state];
                arm_state++;
            }
    }
}

bool WorldCoords::inBounds(const Eigen::Vector3f& pos){
    bool in_bounds;
    Eigen::Vector3f polar = WorldCoords::cartesianToPolar(pos);
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

void WorldCoords::sendPosCmd(){
    swiftpro::position pos;
    pos.x = arm_pos_desired[0] * 1000.0f; // mm
    pos.y = arm_pos_desired[1] * 1000.0f; // mm
    pos.z = arm_pos_desired[2];
    arm_pos_cmd_pub.publish(pos); // send to robot    
}

void WorldCoords::managePump(){
    std_msgs::Bool msg;
    if (arm_state == ArmState::PICK){
        msg.data = true;
    } else msg.data = false;
    pump_pub.publish(msg);
}

void WorldCoords::statusClbk(const swiftpro::SwiftproState& msg){
    arm_pos_actual[0] = msg.x;
    arm_pos_actual[1] = msg.y;
    arm_pos_actual[2] = msg.z;
}

bool WorldCoords::checkReached(){
    for (int i = 0; i < 3; i++){
        if (abs(arm_pos_desired[i] - arm_pos_actual[i]) > pos_error_tolerance){
            return false;
        }
    }
    return true;
}

// void WorldCoords::moveVertical(){
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

Eigen::Vector3f WorldCoords::transformCamToArm(const Eigen::Vector3f& cam){
    return R * cam + t; 
}

void WorldCoords::processActionClbk(const lfm::Action& msg){
    Eigen::Vector3f hover_start, drop, pick, release, hover_end, clear; 
    int tag_id = msg.target_tag;
    Eigen::Vector3f coords = WorldCoords::getTagCoords(tag_id);
    hover_start[0] = coords[0];
    hover_start[1] = coords[1];
    hover_start[2] = standoff_height;
    drop = hover_start;
    drop[2] = pick_height;
    pick = drop;
    release = WorldCoords::calcReleaseCoords(tag_id, msg.dist, msg.angle);
    hover_end = release;
    hover_end[2] = standoff_height;
    clear = hover_end;
    clear[2] = clear_height;
    arm_pos_sequence = {hover_start, drop, pick, release, hover_end, clear};
    for (auto pos : arm_pos_sequence){
        if (!WorldCoords::inBounds(pos)){
            std::cout<<"Illegal move requested. Try again!"<<std::endl;
            arm_state = ArmState::IDLE;
            arm_pos_sequence.clear();
            break;
        }
    }
    arm_state = ArmState::HOVER_START; // if all clear, move to the first position!
}

Eigen::Vector3f WorldCoords::getTagCoords(const int& tag_id){
    Eigen::Vector3f coords;
    if (tag_centers_3d_cam.find(tag_id) != tag_centers_3d_cam.end()){
        coords = WorldCoords::transformCamToArm(tag_centers_3d_cam.find(tag_id)->second);
    }
    return coords;
}

Eigen::Vector3f WorldCoords::calcReleaseCoords(const int& tag_id, const float& dist, const float& angle){
    Eigen::Vector3f start, end; 
    start = WorldCoords::getTagCoords(tag_id);
    float dx = dist * cos(angle * M_PI / 180.0);
    float dy = dist * sin(angle * M_PI / 180.0);
    end[0] = start[0] + dx;
    end[1] = start[1] + dy;
    end[2] = start[2];
    return end;
}

Eigen::Vector3f WorldCoords::cartesianToPolar(const Eigen::Vector3f& pos){
    Eigen::Vector3f polar; // {R, th, z}
    polar[0] = sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
    polar[1] = atan2(pos[1], pos[0]);
    polar[2] = pos[2];
    return polar;
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
            qbar += it_arm->second;
        }

        // calc weighted centroids
        pbar /= num_points;
        qbar /= num_points;
        
        std::cout<<pbar<<std::endl;        
        std::cout<<qbar<<std::endl;        

        // calc centered vectors
        for (int i = 0; i < num_points; i++){
            auto it_cam = cam.find(i);
            auto it_arm = arm.find(i);
            X.col(i) << it_cam->second - pbar;
            Y.col(i) << it_arm->second - qbar;
        }
        std::cout<<"X: \n";
        std::cout<<X<<std::endl;
        std::cout<<"Y: \n";
        std::cout<<Y<<std::endl;
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

        std::cout<<"R:"<<R<<std::endl;
        std::cout<<"t:"<<t<<std::endl;

        return true;
    } else return false;

}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "lfm");
    WorldCoords wc;
    wc.run();
	return 0;
}