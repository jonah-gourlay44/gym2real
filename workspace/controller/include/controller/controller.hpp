#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>
#include <stdio.h>
#include <assert.h>
#include <controller/transform.hpp>

/** Controllers should be defined with relevant transforms/remaps
 *  Example:
 * 
 *  `
 *  - Twip1:
 *      observations: 4 # Number of observations
 *      actions: 2 # Number of actions
 *      type: onnx
 *      model_path: cfg/Twip.pth.onnx
 *  `
 */
class BaseController
{
public:
    BaseController(
        int observations, 
        int actions) :
        observations_(observations), 
        actions_(actions)
    {
        input_buffer_ = new float[observations]();
        output_buffer_ = new float[actions]();
    };

    ~BaseController(){
        delete[] input_buffer_;
        delete[] output_buffer_;
    };

    int run()
    {
        //Pre-transforms (load data into buffers)
        for (auto &trans : pre_transforms_)
        {
            trans.apply();
        }

        //Main control step (inference)
        control_step();

        //Post-transforms (write data to buffers)
        for (auto &trans : post_transforms_)
        {
            trans.apply();
        }
        return 0;
    };

    int addTransforms(std::vector<TransformRule<float, float>> pre_transforms, std::vector<TransformRule<float, float>> post_transforms){
        pre_transforms_ = pre_transforms;
        post_transforms_ = post_transforms;
        return 0;
    };

    std::pair<float*,float*> getBuffers(){
        return std::pair<float*,float*>(input_buffer_,output_buffer_);
    };
protected:
    virtual int init(){return 0;};
    virtual int control_step(){return 0;};

    int observations_;
    int actions_;
    std::vector<TransformRule<float, float>> pre_transforms_;
    std::vector<TransformRule<float, float>> post_transforms_;

    float *input_buffer_ = nullptr;
    float *output_buffer_ = nullptr;
};


class OnnxController : public BaseController
{
public:
    OnnxController(
        int observations, 
        int actions,
        std::string model_path
        ) : model_path_(model_path), BaseController(observations,actions)
    {
        init();
    };

    int init()
    {
        Ort::SessionOptions options;
        OrtCUDAProviderOptions cuda_options;
        // Have to manually initialize options in older versions of OnnxRuntime
        cuda_options.device_id = 0;
        cuda_options.cudnn_conv_algo_search = EXHAUSTIVE;
        cuda_options.gpu_mem_limit = SIZE_MAX;
        cuda_options.arena_extend_strategy = 0;
        cuda_options.do_copy_in_default_stream=0;
        cuda_options.has_user_compute_stream = 0;
        cuda_options.user_compute_stream = nullptr;
        cuda_options.default_memory_arena_cfg = nullptr;
        options.AppendExecutionProvider_CUDA(cuda_options);

        session_ = Ort::Session{env_, model_path_.c_str(), options};
        input_shape_ = {observations_};
        output_shape_ = {actions_};

        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        input_tensor_ = Ort::Value::CreateTensor<float>(memory_info, input_buffer_, observations_, input_shape_.data(), input_shape_.size());
        output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, output_buffer_, actions_, output_shape_.data(), output_shape_.size());
        return 0;
    };

protected:
    int control_step()
    {
        const char *input_names[] = {"observations"};
        const char *output_names[] = {"actions"};

        session_.Run(opt_, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);

        return 0;
    }

private:
    std::string model_path_;
    Ort::Env env_;
    Ort::Session session_{nullptr};
    Ort::RunOptions opt_{nullptr};

    Ort::Value input_tensor_{nullptr};
    std::array<int64_t, 1> input_shape_;

    Ort::Value output_tensor_{nullptr};
    std::array<int64_t, 1> output_shape_;
};