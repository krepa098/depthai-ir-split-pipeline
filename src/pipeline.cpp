#include "oak/pipeline.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pipeline
{
    const int pool_size = 24;
    const float fps = 30.0f;
    const int enc_quality = 80;

    std::tuple<std::vector<std::pair<float, float>>, int, int> get_mesh(std::shared_ptr<dai::Device> &device, dai::CameraBoardSocket socket, int w, int h)
    {

        auto cal = device->readCalibration2();

        auto m1 = cal.getCameraIntrinsics(socket, std::tuple{w, h});

        // flatten the vector<vector<f32>>
        int rows = m1.size();
        int cols = m1[0].size();

        std::vector<float> flatVec;
        for (const auto &row : m1)
        {
            flatVec.insert(flatVec.end(), row.begin(), row.end());
        }
        cv::Mat M1(flatVec, true);
        M1 = M1.reshape(0, rows);

        auto d1 = cv::Mat(cal.getDistortionCoefficients(socket), true);

        cv::Mat map_x;
        cv::Mat map_y;
        // cv::Mat M1 = cv::getOptimalNewCameraMatrix(M1, d1, cv::Size(w, h), 0);
        cv::initUndistortRectifyMap(M1, d1, cv::Mat(), M1, cv::Size(w, h), CV_32FC1, map_x, map_y);

        const int mesh_cell_size = 16;
        std::vector<std::pair<float, float>> mesh;

        int mesh_width = 0;
        int mesh_height = 0;

        for (int y = 0; y <= map_x.rows; ++y) // over the height
        {
            if (y % mesh_cell_size == 0)
            {
                mesh_height++;
                // std::vector<float> row_left;
                for (int x = 0; x < map_x.cols; ++x) // over the width
                {
                    if (x % mesh_cell_size == 0)
                    {
                        mesh_width++;
                        if (y == map_x.rows && x == map_x.cols)
                        {
                            mesh.push_back({map_x.at<float>(y - 1, x - 1),
                                            map_y.at<float>(y - 1, x - 1)});
                        }
                        else if (y == map_x.rows)
                        {
                            mesh.push_back({map_x.at<float>(y - 1, x),
                                            map_y.at<float>(y - 1, x)});
                        }
                        else if (x == map_x.rows)
                        {
                            mesh.push_back({map_x.at<float>(y, x - 1),
                                            map_y.at<float>(y, x - 1)});
                        }
                        else
                        {
                            mesh.push_back({map_x.at<float>(y, x),
                                            map_y.at<float>(y, x)});
                        }
                    }
                }

                if ((map_x.cols % mesh_cell_size) % 2 != 0)
                {
                    mesh.push_back({0.0, 0.0});
                }
            }
        }
        mesh_width /= mesh_height;

        return {mesh, mesh_width, mesh_height};
    }

    dai::Pipeline create_pipeline(std::shared_ptr<dai::Device> &device, PipelineInfo &pipeline_info, bool rectify_mono)
    {
        auto cal = device->readCalibration2();

        dai::Pipeline pipeline;

        // sys logger (cpu usage, temperature etc.)
        auto sys_logger = pipeline.create<dai::node::SystemLogger>();

        auto imu = pipeline.create<dai::node::IMU>();
        imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 400);
        imu->setBatchReportThreshold(1);
        imu->setMaxBatchReports(10);

        // color
        // auto color = pipeline.create<dai::node::ColorCamera>();
        // color->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        // color->setInterleaved(false);
        // color->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
        // color->setBoardSocket(dai::CameraBoardSocket::CAM_A);
        // color->setFps(fps);
        // color->setPreviewKeepAspectRatio(false);
        // color->setIspScale(4, 6);
        // color->initialControl.setManualFocus(128);

        auto color = pipeline.create<dai::node::Camera>();
        color->setSize(1280, 720);
        color->setMeshSource(dai::CameraProperties::WarpMeshSource::CALIBRATION);
        color->setFps(fps);
        color->setBoardSocket(dai::CameraBoardSocket::CAM_A);
        color->initialControl.setManualFocus(cal.getLensPosition(dai::CameraBoardSocket::CAM_A));

        // color->setIspNumFramesPool(pool_size);
        // color->setVideoNumFramesPool(pool_size);

        // color encoder
        auto color_enc = pipeline.create<dai::node::VideoEncoder>();
        color_enc->setDefaultProfilePreset(color->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
        color_enc->setQuality(enc_quality);
        color->video.link(color_enc->input);

        pipeline_info.mono_res = color->getSize();

        // depth / stereo
        auto mono_res = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        auto mono_left = pipeline.create<dai::node::MonoCamera>();
        mono_left->setResolution(mono_res);
        mono_left->setBoardSocket(dai::CameraBoardSocket::CAM_B);
        mono_left->setFps(fps);
        mono_left->setNumFramesPool(pool_size);

        auto mono_right = pipeline.create<dai::node::MonoCamera>();
        mono_right->setResolution(mono_res);
        mono_right->setBoardSocket(dai::CameraBoardSocket::CAM_C);
        mono_right->setFps(fps);
        mono_right->setNumFramesPool(pool_size);

        pipeline_info.mono_res = mono_right->getResolutionSize();

        auto stereo = pipeline.create<dai::node::StereoDepth>();
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
        stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_3x3); // in hardware -> fast
        stereo->setRectification(true);
        stereo->setExtendedDisparity(false);
        stereo->setSubpixel(true);
        stereo->setLeftRightCheck(true);
        stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A); // rgb
        stereo->setNumFramesPool(pool_size);

        // manip
        auto manip = pipeline.create<dai::node::ImageManip>();

        if (rectify_mono)
        {
            auto [mesh, w, h] = get_mesh(device, dai::CameraBoardSocket::CAM_C, mono_right->getResolutionWidth(), mono_right->getResolutionHeight());
            manip->setWarpMesh(mesh, w, h);
        }

        // script
        auto script = pipeline.create<dai::node::Script>();
        script->setProcessor(dai::ProcessorType::LEON_CSS);
        script->setScript(R"(
dotBright = 0.1
floodBright = 0.1
LOGGING = False  # Set `True` for latency/timings debugging

node.warn(f'IR drivers detected: {str(Device.getIrDrivers())}')

flagDot = False
while True:
    # Wait first for a frame event, received at MIPI start-of-frame
    event = node.io['event'].get()
    if LOGGING: tEvent = Clock.now()

    # Immediately reconfigure the IR driver.
    # Note the logic is inverted, as it applies for next frame
    Device.setIrLaserDotProjectorIntensity(0 if flagDot else dotBright)
    Device.setIrFloodLightIntensity(floodBright)
    if LOGGING: tIrSet = Clock.now()

    # Wait for the actual frames (after MIPI capture and ISP proc is done)
    frameL = node.io['frameL'].get()
    if LOGGING: tLeft = Clock.now()
    frameR = node.io['frameR'].get()
    if LOGGING: tRight = Clock.now()
                                
    if LOGGING:
        latIR      = (tIrSet - tEvent               ).total_seconds() * 1000
        latEv      = (tEvent - event.getTimestamp() ).total_seconds() * 1000
        latProcL   = (tLeft  - event.getTimestamp() ).total_seconds() * 1000
        diffRecvRL = (tRight - tLeft                ).total_seconds() * 1000
        node.warn(f'T[ms] latEv:{latEv:5.3f} latIR:{latIR:5.3f} latProcL:{latProcL:6.3f} '
                + f' diffRecvRL:{diffRecvRL:5.3f}')

    # Sync checks
    diffSeq = frameL.getSequenceNum() - event.getSequenceNum()
    diffTsEv = (frameL.getTimestamp() - event.getTimestamp()).total_seconds() * 1000
    diffTsRL = (frameR.getTimestamp() - frameL.getTimestamp()).total_seconds() * 1000
    if diffSeq or diffTsEv or (abs(diffTsRL) > 0.8):
        node.error(f'frame/event desync! Fr-Ev: {diffSeq} frames,'
                + f' {diffTsEv:.3f} ms; R-L: {diffTsRL:.3f} ms')
        frameR = None
        frameL = None
        continue

    # Route the frames to their respective outputs
    if flagDot:
        node.io['dotL'].send(frameL)

    node.io['dotR' if flagDot else 'floodR'].send(frameR)

    flagDot = not flagDot
        )");

        // script inputs
        mono_left->frameEvent.link(script->inputs["event"]);
        mono_left->out.link(script->inputs["frameL"]);
        mono_right->out.link(script->inputs["frameR"]);

        // right mono encoder
        auto mono_enc = pipeline.create<dai::node::VideoEncoder>();
        mono_enc->setDefaultProfilePreset(mono_right->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
        mono_enc->setQuality(enc_quality);

        auto mono_out = pipeline.create<dai::node::XLinkOut>();
        mono_out->setStreamName("mono");

        // script outputs
        script->outputs["floodR"].link(manip->inputImage);
        script->outputs["dotL"].link(stereo->left);
        script->outputs["dotR"].link(stereo->right);

        manip->out.link(mono_enc->input);

        // sync
        auto sync = pipeline.create<dai::node::Sync>();
        sync->setSyncThreshold(std::chrono::milliseconds(200));
        sync->setSyncAttempts(2);

        color_enc->out.link(sync->inputs["color"]);
        stereo->depth.link(sync->inputs["depth"]);

        // outputs
        auto sys_logger_out = pipeline.create<dai::node::XLinkOut>();
        sys_logger_out->setStreamName("sys_logger");
        auto imu_out = pipeline.create<dai::node::XLinkOut>();
        imu_out->setStreamName("imu");
        auto mux_out = pipeline.create<dai::node::XLinkOut>();
        mux_out->setStreamName("mux");

        sync->out.link(mux_out->input);

        sys_logger->out.link(sys_logger_out->input);
        mono_enc->out.link(mono_out->input);
        imu->out.link(imu_out->input);

        return pipeline;
    }

} // namespace