#include <controller/transform.hpp>
#include <controller/controller.hpp>

using namespace std;
int main(int argc, char **argv)
{   
    // Setup buffers
    // TODO: ROS
    float quaternion_tmp[4] = {0, 0, 0, 1};
    float position_tmp[2] = {0., 1.};
    float motor_command[2] = {0, 0};

    // Create controller and get input/output buffers
    OnnxController model_controller(3, 1, "Twip.pth.onnx");
    auto buffers = model_controller.getBuffers();

    // Controller can execute transforms before and after control step
    vector<TransformRule<float, float>> pre_transforms;
    vector<TransformRule<float, float>> post_transforms;

    // Define pre-transforms (transforms needed before control step)
    // bservation 0 is pitch
    auto quaternion_to_pitch = [](float *in_ptr, float *out_ptr)
    {
        float &x = in_ptr[0];
        float &y = in_ptr[1];
        float &z = in_ptr[2];
        float &w = in_ptr[3];
        out_ptr[0] = atan2(2 * x * w + 2 * z * y, 1 - 2 * x * x - 2 * y * y);
    };
    RemapRule<float, float> quaternion_rule(quaternion_to_pitch);
    pre_transforms.emplace_back(quaternion_tmp, buffers.first, quaternion_rule);

    // Observation 1 is last action
    RangeRemapRule<float, float> last_action_rule(pair<float, float>(-1., 1.), pair<float, float>(-1., 1.), {0}, {1}, false);
    pre_transforms.emplace_back(buffers.second, buffers.first, last_action_rule);

    // Observation 2 is average of encoder positions
    auto tanh_avg = [](float *in_ptr, float *out_ptr)
    {
        out_ptr[0] = tanh((in_ptr[0] + in_ptr[1]) / 2.);
    };
    RemapRule<float, float> tanh_avg_rule(tanh_avg);
    pre_transforms.emplace_back(position_tmp, buffers.first, tanh_avg_rule);

    // Define post-transforms (transforms needed after control step)
    // Action 0 can be scaled to get our motor output
    float max_velocity = 17.8;
    float output_scale = 1.5;
    RangeRemapRule<float, float> motor_command_rule(
        pair<float, float>(-1., 1.),
        pair<float, float>(-max_velocity * output_scale, max_velocity * output_scale),
        {0, 0},
        {0, 1},
        false);
    post_transforms.emplace_back(buffers.second, motor_command, motor_command_rule);

    //Now that setup is done, we can run the control step
    model_controller.run();


    cout<<buffers.second[0]<<endl;

    return 0;
}
