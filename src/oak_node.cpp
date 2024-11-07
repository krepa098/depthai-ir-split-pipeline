#include "rclcpp/rclcpp.hpp"
#include "depthai/depthai.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

#include "camera_info_manager/camera_info_manager.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "imu_filter_madgwick/imu_filter_ros.h"

#include "oak/pipeline.h"

const std::string FRAME_RGB = "oak_rgb_camera_optical_frame";
const std::string FRAME_STEREO = "oak_rgb_camera_optical_frame";
const std::string FRAME_RIGHT_MONO = "oak_right_camera_optical_frame";
const std::string FRAME_LEFT_MONO = "oak_right_camera_optical_frame";

class Node : public rclcpp::Node
{
public:
    Node() : rclcpp::Node("oak"), imu_filter_(this->get_node_options())
    {
        color_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "~/rgb/image_raw/compressed", rclcpp::SystemDefaultsQoS());

        right_mono_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "~/right/image_raw/compressed", rclcpp::SystemDefaultsQoS());

        stereo_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "~/stereo/image_raw", rclcpp::SystemDefaultsQoS());

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "~/imu", rclcpp::SystemDefaultsQoS());

        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/rgb/camera_info", rclcpp::SystemDefaultsQoS());

        right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/right/camera_info", rclcpp::SystemDefaultsQoS());

        stereo_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/stereo/camera_info", rclcpp::SystemDefaultsQoS());

        img_converter_stereo_ = std::make_shared<dai::rosBridge::ImageConverter>(FRAME_STEREO, false);
        img_converter_mono_ = std::make_shared<dai::rosBridge::ImageConverter>(FRAME_RIGHT_MONO, false);
        img_converter_rgb_ = std::make_shared<dai::rosBridge::ImageConverter>(FRAME_RGB, false);
        imu_converter_ = std::make_shared<dai::rosBridge::ImuConverter>("imu");

        RCLCPP_INFO(get_logger(), "opening device");
        device_ = std::make_shared<dai::Device>();

        RCLCPP_INFO(get_logger(), "creating pipeline");
        pipeline::PipelineInfo pipline_info;
        auto pipeline = pipeline::create_pipeline(device_, pipline_info);

        // RCLCPP_INFO_STREAM(get_logger(), "Pipeline.json" << std::endl
        //                                                  << pipeline.serializeToJson());

        RCLCPP_INFO(get_logger(), "start pipeline");
        device_->startPipeline(pipeline);

        // camera info
        // using depthai_ros_driver::dai_nodes::sensor_helpers;
        auto calibration_handler = device_->readCalibration();

        info_manager_rgb_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "oak");
        auto rgb_info = img_converter_rgb_->calibrationToCameraInfo(calibration_handler,
                                                                    dai::CameraBoardSocket::CAM_A,
                                                                    std::get<0>(pipline_info.color_res),
                                                                    std::get<1>(pipline_info.color_res));
        info_manager_rgb_->setCameraInfo(rgb_info);

        info_manager_right_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "oak");
        auto right_info = img_converter_mono_->calibrationToCameraInfo(calibration_handler,
                                                                       dai::CameraBoardSocket::CAM_C,
                                                                       std::get<0>(pipline_info.mono_res),
                                                                       std::get<1>(pipline_info.mono_res));
        info_manager_right_->setCameraInfo(right_info);

        RCLCPP_INFO(get_logger(), "creating output queues");
        sys_logger_queue_ = device_->getOutputQueue("sys_logger", 3, false);
        mono_queue_ = device_->getOutputQueue("mono", 3, false);
        mux_queue_ = device_->getOutputQueue("mux", 3, false);
        imu_queue_ = device_->getOutputQueue("imu", 3, false);

        // callbacks
        sys_logger_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) { //
        });

        mux_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) { //
            if (const auto group = std::dynamic_pointer_cast<dai::MessageGroup>(callback))
            {
                if (const auto color = group->get<dai::EncodedFrame>("color"))
                {
                    auto msg = img_converter_rgb_->toRosFFMPEGPacket(color);

                    sensor_msgs::msg::CompressedImage img;
                    img.data = msg.data;
                    img.format = msg.encoding;
                    img.header.frame_id = msg.header.frame_id;
                    img.header.stamp = msg.header.stamp;
                    color_image_pub_->publish(img);
                    camera_info_pub_->publish(info_manager_rgb_->getCameraInfo());
                }

                if (const auto depth = group->get<dai::ImgFrame>("depth"))
                {
                    auto msg = img_converter_stereo_->toRosMsgRawPtr(depth);
                    stereo_image_pub_->publish(msg);
                    stereo_info_pub_->publish(info_manager_right_->getCameraInfo());
                }
            }
        });

        // right mono (compressed)
        mono_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) { //
            if (const auto buf = std::dynamic_pointer_cast<dai::EncodedFrame>(callback))
            {
                auto msg = img_converter_stereo_->toRosFFMPEGPacket(buf);

                sensor_msgs::msg::CompressedImage img;
                img.data = msg.data;
                img.format = msg.encoding;
                img.header.frame_id = msg.header.frame_id;
                img.header.stamp = msg.header.stamp;
                right_mono_image_pub_->publish(img);

                right_info_pub_->publish(info_manager_right_->getCameraInfo());
            }
        });

        // imu
        imu_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) { //
            if (const auto buf = std::dynamic_pointer_cast<dai::IMUData>(callback))
            {
                std::deque<sensor_msgs::msg::Imu> imu_msgs;
                imu_converter_->toRosMsg(buf, imu_msgs);

                for (auto msg : imu_msgs)
                {
                    imu_pub_->publish(msg);
                    imu_filter_.imuCallback(std::shared_ptr<sensor_msgs::msg::Imu>(new sensor_msgs::msg::Imu(msg)));
                }
            }
        });
    }

    std::shared_ptr<dai::Device> device()
    {
        return device_;
    }

public:
    std::shared_ptr<dai::Device> device_;

    std::shared_ptr<dai::DataOutputQueue> sys_logger_queue_;
    std::shared_ptr<dai::DataOutputQueue> mono_queue_;
    std::shared_ptr<dai::DataOutputQueue> imu_queue_;
    std::shared_ptr<dai::DataOutputQueue> mux_queue_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> color_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> right_mono_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> stereo_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> camera_info_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> right_info_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> stereo_info_pub_;

    std::shared_ptr<dai::rosBridge::ImageConverter> img_converter_rgb_;
    std::shared_ptr<dai::rosBridge::ImageConverter> img_converter_stereo_;
    std::shared_ptr<dai::rosBridge::ImageConverter> img_converter_mono_;
    std::shared_ptr<dai::rosBridge::ImuConverter> imu_converter_;

    ImuFilterMadgwickRos imu_filter_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_rgb_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_right_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();

    // tf
    auto calibration_handler = node->device()->readCalibration();
    auto camera_features = node->device()->getConnectedCameraFeatures();
    auto tf_publisher = std::make_shared<dai::rosBridge::TFPublisher>(node, calibration_handler, camera_features, "oak", "oak-d-pro", "oak", "oak", "0", "0", "0", "0", "0", "0", "", "", "", false);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}