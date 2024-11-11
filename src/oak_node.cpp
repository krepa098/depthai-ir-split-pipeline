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

// #include "oak/codec.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/publisher_plugin.hpp"
#include "pluginlib/class_loader.hpp"

#include "oak/pipeline.h"

const std::string FRAME_RGB = "oak_rgb_camera_optical_frame";
const std::string FRAME_STEREO = "oak_rgb_camera_optical_frame";
const std::string FRAME_RIGHT_MONO = "oak_right_camera_optical_frame";
const std::string FRAME_LEFT_MONO = "oak_right_camera_optical_frame";
const std::string FRAME_IMU = "oak_imu_frame";

typedef image_transport::PublisherPlugin Plugin;

pluginlib::ClassLoader<Plugin> img_transport_loader(
    "image_transport",
    "image_transport::PublisherPlugin");

class Node : public rclcpp::Node
{
public:
    Node() : rclcpp::Node("oak")
    {
        declare_parameter<float>("fps");
        declare_parameter<int>("pool_size");
        declare_parameter<int>("encoder_quality");
        declare_parameter<float>("floodlight_intensity");
        declare_parameter<float>("dot_intensity");

        // load the image transport plugin

        std::string lookup_name = Plugin::getLookupName("compressedDepth");
        cdp_ = img_transport_loader.createSharedInstance(lookup_name);
        cdp_->advertise(this, "~/stereo/image_raw");

        pipeline::PipelineOptions options;

        get_parameter("fps", options.fps);
        get_parameter("pool_size", options.pool_size);
        get_parameter("encoder_quality", options.encoder_quality);
        get_parameter("floodlight_intensity", options.floodlight_intensity);
        get_parameter("dot_intensity", options.dot_intensity);

        color_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "~/rgb/image_raw/compressed", rclcpp::SystemDefaultsQoS());

        color_image_rect_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "~/rgb/image_rect/compressed", rclcpp::SystemDefaultsQoS());

        right_mono_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "~/right/image_raw/compressed", rclcpp::SystemDefaultsQoS());

        right_mono_rect_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "~/right/image_rect/compressed", rclcpp::SystemDefaultsQoS());

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "~/imu/data", rclcpp::SystemDefaultsQoS());

        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/rgb/camera_info", rclcpp::SystemDefaultsQoS());

        right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/right/camera_info", rclcpp::SystemDefaultsQoS());

        stereo_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/stereo/camera_info", rclcpp::SystemDefaultsQoS());

        img_converter_stereo_ = std::make_shared<dai::rosBridge::ImageConverter>(FRAME_STEREO, false);
        img_converter_mono_ = std::make_shared<dai::rosBridge::ImageConverter>(FRAME_RIGHT_MONO, false);
        img_converter_rgb_ = std::make_shared<dai::rosBridge::ImageConverter>(FRAME_RGB, false);
        imu_converter_ = std::make_shared<dai::rosBridge::ImuConverter>(FRAME_IMU);

        RCLCPP_INFO(get_logger(), "opening device");
        device_ = std::make_shared<dai::Device>();

        RCLCPP_INFO(get_logger(), "creating pipeline");
        pipeline::PipelineInfo pipline_info;
        auto pipeline = pipeline::create_pipeline(device_, options, pipline_info);

        RCLCPP_DEBUG_STREAM(get_logger(), "Pipeline.json" << std::endl
                                                          << pipeline.serializeToJson());

        RCLCPP_INFO(get_logger(), "start pipeline");
        device_->startPipeline(pipeline);

        // camera info
        auto calibration_handler = device_->readCalibration2();

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
        mono_queue_ = device_->getOutputQueue("mono", 3, false);
        mono_rect_queue_ = device_->getOutputQueue("mono_rect", 3, false);
        mux_queue_ = device_->getOutputQueue("mux", 3, false);
        imu_queue_ = device_->getOutputQueue("imu", 3, false);

        // callbacks
        mux_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) { //
            if (const auto group = std::dynamic_pointer_cast<dai::MessageGroup>(callback))
            {
                if (const auto color = group->get<dai::EncodedFrame>("color"))
                {
                    auto msg = img_converter_rgb_->toRosFFMPEGPacket(color);

                    sensor_msgs::msg::CompressedImage img;
                    img.data = msg.data;
                    img.format = msg.encoding;
                    img.header = msg.header;
                    if (color_image_pub_->get_subscription_count() > 0)
                        color_image_pub_->publish(img);

                    auto cam_info = info_manager_rgb_->getCameraInfo();
                    cam_info.header = img.header;
                    if (camera_info_pub_->get_subscription_count() > 0)
                        camera_info_pub_->publish(cam_info);
                }

                if (const auto color = group->get<dai::EncodedFrame>("color_rect"))
                {
                    if (color_image_rect_pub_->get_subscription_count() > 0)
                    {
                        auto msg = img_converter_rgb_->toRosFFMPEGPacket(color);
                        sensor_msgs::msg::CompressedImage img;
                        img.data = msg.data;
                        img.format = msg.encoding;
                        img.header = msg.header;
                        color_image_rect_pub_->publish(img);
                    }
                }

                if (const auto depth = group->get<dai::ImgFrame>("depth"))
                {
                    if (cdp_->getNumSubscribers() > 0)
                    {
                        auto msg = img_converter_stereo_->toRosMsgPtr(depth);
                        cdp_->publish(*msg);
                        auto cam_info = info_manager_right_->getCameraInfo();
                        cam_info.header = msg->header;
                        if (stereo_info_pub_->get_subscription_count() > 0)
                            stereo_info_pub_->publish(cam_info);
                    }
                }
            }
        });

        // right mono (compressed)
        mono_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) { //
            if (const auto buf = std::dynamic_pointer_cast<dai::EncodedFrame>(callback))
            {
                auto msg = img_converter_mono_->toRosFFMPEGPacket(buf);

                sensor_msgs::msg::CompressedImage img;
                img.data = msg.data;
                img.format = msg.encoding;
                img.header = msg.header;
                if (right_mono_image_pub_->get_subscription_count() > 0)
                    right_mono_image_pub_->publish(img);

                auto cam_info = info_manager_right_->getCameraInfo();
                cam_info.header = img.header;
                if (right_info_pub_->get_subscription_count() > 0)
                    right_info_pub_->publish(cam_info);
            }
        });

        // right mono rect (compressed)
        mono_rect_queue_->addCallback([&](std::shared_ptr<dai::ADatatype> callback) { //
            if (const auto buf = std::dynamic_pointer_cast<dai::EncodedFrame>(callback))
            {
                auto msg = img_converter_mono_->toRosFFMPEGPacket(buf);

                sensor_msgs::msg::CompressedImage img;
                img.data = msg.data;
                img.format = msg.encoding;
                img.header = msg.header;
                if (right_mono_rect_image_pub_->get_subscription_count() > 0)
                    right_mono_rect_image_pub_->publish(img);
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
                    if (imu_pub_->get_subscription_count() > 0)
                        imu_pub_->publish(msg);
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

    std::shared_ptr<dai::DataOutputQueue> mono_queue_;
    std::shared_ptr<dai::DataOutputQueue> mono_rect_queue_;
    std::shared_ptr<dai::DataOutputQueue> imu_queue_;
    std::shared_ptr<dai::DataOutputQueue> mux_queue_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> color_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> color_image_rect_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> right_mono_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> right_mono_rect_image_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> camera_info_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> right_info_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> stereo_info_pub_;

    std::shared_ptr<dai::rosBridge::ImageConverter> img_converter_rgb_;
    std::shared_ptr<dai::rosBridge::ImageConverter> img_converter_stereo_;
    std::shared_ptr<dai::rosBridge::ImageConverter> img_converter_mono_;
    std::shared_ptr<dai::rosBridge::ImuConverter> imu_converter_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_rgb_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_right_;

    std::shared_ptr<Plugin> cdp_;
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();

    // tf
    auto calibration_handler = node->device()->readCalibration2();
    auto camera_features = node->device()->getConnectedCameraFeatures();
    auto tf_publisher = std::make_shared<dai::rosBridge::TFPublisher>(node, calibration_handler, camera_features, "oak", "oak-d-pro", "oak", "oak", "0", "0", "0", "0", "0", "0", "", "", "", false);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}