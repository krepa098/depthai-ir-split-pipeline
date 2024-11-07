#pragma once

#include "depthai/depthai.hpp"

namespace pipeline
{
    struct PipelineInfo
    {
        std::tuple<int, int> mono_res;
        std::tuple<int, int> color_res;
    };

    dai::Pipeline create_pipeline(std::shared_ptr<dai::Device> &device, PipelineInfo &pipeline_info, bool rectify_mono);
};