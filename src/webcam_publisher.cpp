#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/SetCameraInfo.h>

bool setCameraInfoCallback(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res,
                           camera_info_manager::CameraInfoManager &cam_info_manager) {
    if (cam_info_manager.setCameraInfo(req.camera_info)) {
        res.success = true;
        res.status_message = "Camera info updated successfully.";
        ROS_INFO("Camera info updated.");
    } else {
        res.success = false;
        res.status_message = "Failed to update camera info.";
        ROS_ERROR("Failed to update camera info.");
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "webcam_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);
    ros::Publisher pub_info = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);

    // Camera info manager
    camera_info_manager::CameraInfoManager cam_info_manager(nh, "narrow_stereo", "file:///home/volvo4/catkin_ws/src/turtlebot3_autorace_2020/turtlebot3_autorace_camera/calibration/intrinsic_calibration/camera.yaml");

    // Service for setting camera info
    ros::ServiceServer set_camera_info_srv = nh.advertiseService<sensor_msgs::SetCameraInfo::Request, sensor_msgs::SetCameraInfo::Response>(
        "/camera/set_camera_info", boost::bind(setCameraInfoCallback, _1, _2, boost::ref(cam_info_manager)));

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        ROS_ERROR("failed to open webcam");
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            ROS_WARN("empty frame");
            continue;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);

         // Publish the camera info
        sensor_msgs::CameraInfo cam_info = cam_info_manager.getCameraInfo();
        cam_info.header = msg->header;  // Synchronize headers
        pub_info.publish(cam_info);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
