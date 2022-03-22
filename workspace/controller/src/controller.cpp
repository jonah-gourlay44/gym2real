#include <controller/transform.hpp>
#include <controller/controller.hpp>
#include <controller/memory_manager.hpp>

using namespace std;
int main(int argc, char **argv)
{      
    // Create memory manager (creates buffers and controllers from config.yaml) and lock memory
    MemoryManager memory("cfg/config.yaml");

    // Get shared memory buffers
    float * imu_quaternion = nullptr;
    memory.create_buffer<float>("imu_quaternion", 1, imu_quaternion, 16);
    float *motor_position = nullptr;
    memory.create_buffer<float>("motor_position", 1, motor_position, 8);
    float * motor_command = nullptr;
    memory.create_buffer<float>("motor_command", 1, motor_command, 8);

    // TODO: writing values here for now, need to integrate with ROS
    imu_quaternion[0] = 0;
    imu_quaternion[1] = 0;
    imu_quaternion[2] = 0;
    imu_quaternion[3] = 1;
    motor_position[0] = 0;
    motor_position[1] = 0;
    motor_command[0] = 0;
    motor_command[1] = 0;

    // Get controller and get input/output buffers
    BaseController& model_controller = *memory.getController("Twip1").value();
    auto buffers = model_controller.getBuffers();

    // Controller can execute transforms before and after control step
    vector<TransformRule<>> pre_transforms;
    vector<TransformRule<>> post_transforms;

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
    RemapRule<> quaternion_rule(quaternion_to_pitch);
    pre_transforms.emplace_back(imu_quaternion, buffers.first, quaternion_rule);

    // Observation 1 is last action
    RangeRemapRule<> last_action_rule(pair<float, float>(-1., 1.), pair<float, float>(-1., 1.), {0}, {1}, false);
    pre_transforms.emplace_back(buffers.second, buffers.first, last_action_rule);

    // Observation 2 is average of encoder positions
    auto tanh_avg = [](float *in_ptr, float *out_ptr)
    {
        out_ptr[0] = tanh((in_ptr[0] + in_ptr[1]) / 2.);
    };
    RemapRule<> tanh_avg_rule(tanh_avg);
    pre_transforms.emplace_back(motor_position, buffers.first, tanh_avg_rule);

    // Define post-transforms (transforms needed after control step)
    // Action 0 can be scaled to get our motor output
    float max_velocity = 17.8;
    float output_scale = 1.5;
    RangeRemapRule<> motor_command_rule(
        pair<float, float>(-1., 1.),
        pair<float, float>(-max_velocity * output_scale, max_velocity * output_scale),
        {0, 0},
        {0, 1},
        false);
    post_transforms.emplace_back(buffers.second, motor_command, motor_command_rule);

    // Add transforms to controller
    model_controller.addTransforms(pre_transforms,post_transforms);

    // Now that setup is done, we can run the control step
    model_controller.run();
    auto val = buffers.second[0];
    cout << val << endl;
    cout << motor_command[0] << " " << motor_command[1] << endl;

    return 0;
}
