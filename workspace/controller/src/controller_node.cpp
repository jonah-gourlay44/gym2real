#include <controller/controller_node.hpp>
#include <driver/imu_driver.hpp>
#include <driver/motor_driver.hpp>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

ControllerNode::ControllerNode() : Node("controller"), memory_manager_("cfg/config.yaml")
{
    // Get shared memory buffers
    memory_manager_.create_buffer<float>("imu_quaternion", 1, imu_quaternion_);
    memory_manager_.create_buffer<float>("motor_position", 1, motor_position_);
    memory_manager_.create_buffer<float>("motor_command", 1, motor_command_);

    // Create ROS2 msgs
    imu_state_ = std::make_shared<sensor_msgs::msg::Imu>();
    motor_state_ = std::make_shared<sensor_msgs::msg::JointState>();
    motor_state_->position = {0, 0};
    command_.name = {"motor_l", "motor_r"};
    command_.velocity = {0, 0};

    // Shared memory initializes values to 0, so set any values if needed
    imu_quaternion_[3] = 1;

    RCLCPP_INFO(get_logger(), "1111");
    // Get controller and get input/output buffers
    model_controller_ = memory_manager_.getController("Twip1").value();
    auto buffers = model_controller_->getBuffers();

    // Controller can execute transforms before and after control step
    std::vector<TransformRule<>> pre_transforms;
    std::vector<TransformRule<>> post_transforms;

    // Define pre-transforms (transforms needed before control step)
    // Observation 0 is pitch
    auto quaternion_to_pitch = [](float *in_ptr, float *out_ptr)
    {
        float &x = in_ptr[0];
        float &y = in_ptr[1];
        float &z = in_ptr[2];
        float &w = in_ptr[3];
        out_ptr[0] = atan2(2 * x * w + 2 * z * y, 1 - 2 * x * x - 2 * y * y);
    };
    RemapRule<> quaternion_rule(quaternion_to_pitch);
    pre_transforms.emplace_back(imu_quaternion_, buffers.first, quaternion_rule);

    // Observation 1 is last action
    RangeRemapRule<> last_action_rule(
        {-1, 1},
        {-1, 1},
        {0},
        {1},
        false);
    pre_transforms.emplace_back(buffers.second, buffers.first, last_action_rule);

    // Observation 2 is average of encoder positions (UNUSED)
    /*
    auto tanh_avg = [](float *in_ptr, float *out_ptr)
    {
        out_ptr[0] = tanh((in_ptr[0] + in_ptr[1]) / 2.);
    };
    RemapRule<> tanh_avg_rule(tanh_avg);
    pre_transforms.emplace_back(motor_position_, buffers.first, tanh_avg_rule);
    */

    // Define post-transforms (transforms needed after control step)
    // Action 0 can be scaled to get our motor output
    float max_velocity = 17.8;
    RangeRemapRule<> motor_command_rule(
        {-1, 1},
        {-max_velocity, max_velocity},
        {0, 0},
        {0, 1},
        false);
    post_transforms.emplace_back(buffers.second, motor_command_, motor_command_rule);

    // Add transforms to controller
    model_controller_->addTransforms(pre_transforms, post_transforms);
    RCLCPP_INFO(get_logger(), "1112");
    // Pub/sub setup
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu_data",
        10,
        std::bind(&ControllerNode::imu_callback, this, _1));
    sub_motor_state_ = create_subscription<sensor_msgs::msg::JointState>(
        "motor_state",
        10,
        std::bind(&ControllerNode::motor_state_callback, this, _1));
    pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "motor_command",
        10);
    RCLCPP_INFO(get_logger(), "1121");
    // Set up wall timer
    // TODO: https://github.com/machines-in-motion/real_time_tools

    last_time_ = std::chrono::high_resolution_clock::now();
    auto control_loop_time = 5ms;
    control_loop_timer_ = create_wall_timer(control_loop_time, std::bind(&ControllerNode::control_loop, this));
}

ControllerNode::~ControllerNode()
{
}

void ControllerNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_state_ = std::move(msg);
    imu_quaternion_[0] = imu_state_->orientation.y; // imu Y -> sim X
    imu_quaternion_[1] = imu_state_->orientation.z; // imu Z -> sim Y
    imu_quaternion_[2] = imu_state_->orientation.x; // imu X -> sim Z
    imu_quaternion_[3] = imu_state_->orientation.w;
}

void ControllerNode::motor_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    motor_state_ = std::move(msg);
    motor_position_[0] = motor_state_->position[0];
    motor_position_[1] = motor_state_->position[1];
}

void ControllerNode::motor_command_publish()
{
    command_.header.stamp = get_clock()->now();
    command_.velocity = {motor_command_[0], motor_command_[1]};
    pub_->publish(command_);
}

void ControllerNode::control_loop()
{
    auto now = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count() / 1e6;
    last_time_ = std::chrono::high_resolution_clock::now();
    model_controller_->run(dt);
    RCLCPP_INFO(get_logger(), std::to_string(motor_command_[0]));
    motor_command_publish();    
}

int main(int argc, char **argv)
{   
    // Initialize node
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<ControllerNode>();
    auto motor_driver = std::make_shared<MotorDriver>(32, 33, 11, 23, 12, 24);
    auto imu_driver = std::make_shared<IMUDriver>(19, 3, 5, 0x68);

    // Set real-time priority
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = 98;
    int ret = sched_setscheduler(getpid(), SCHED_FIFO, &param);
    if (ret != 0)
    {
        RCLCPP_ERROR(controller->get_logger(), "Failed to set process priority!");
    }
    else
    {
        // Print thread scheduling priority
        RCLCPP_INFO(controller->get_logger(), "Process priority is " + std::to_string(param.sched_priority));
    }

    // Create static executor
    rclcpp::executors::StaticSingleThreadedExecutor exec;
    exec.add_node(controller->get_node_base_interface());
    exec.add_node(motor_driver->get_node_base_interface());
    exec.add_node(imu_driver->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}