#pragma once

#include "depthai/depthai.hpp"

namespace pipeline
{
    struct PipelineInfo
    {
        std::tuple<int, int> mono_res;
        std::tuple<int, int> color_res;
    };

    struct PipelineOptions
    {
        bool rectify_mono = false;
        float fps = 24.0;
        int pool_size = 24;
        int encoder_quality = 80;
    };

    dai::Pipeline create_pipeline(std::shared_ptr<dai::Device> &device, PipelineOptions options, PipelineInfo &pipeline_info);
};