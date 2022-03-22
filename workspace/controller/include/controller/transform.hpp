#pragma once
/** Remaps copy data from one buffer to another and remaps/inverts/clips based on configuration.
 *  Data in the memory stores should always be in the same space for easier troubleshooting.
 *
 *  Examples:
 *
 *   Interfacing with actuators (assumes actuator commands and feedback are in same space):
 *   `
 *   - actuator_1: # Actuator space (depends on actuator firmware) to joint space (URDF)
 *       actuator_range: [-1, 1] # Values that should be given to motor interface
 *       joint: joint_1 # Gets limits from URDF
 *       invert: True
 *       type: effort # position/velocity/effort
 *
 *   - actuator_2: # Actuator space (depends on actuator firmware) to joint space (URDF)
 *       actuator_range: [-1, 1] # Values that should be given to motor interface
 *       joint_range: [0, 3.14159] # Overrides limits from URDF
 *       invert: False
 *       type: effort # position/velocity/effort
 *   `
 *
 *   Interfacing with sensors:
 *   `
 *   - sensor_1: # Sensor space
 *       sensor_range: [0, 255]
 *       controller_range: [0, 1]
 *       invert: False
 *   `
 **/
#include <string>
#include <vector>
#include <functional>
//#include <urdf_parser/urdf_parser.h>

// enum ControlType
// {
//     CONTROL_POSITION,
//     CONTROL_VELOCITY,
//     CONTROL_EFFORT
// };

template <class T1 = float, class T2 = float>
struct RemapRule
{
    RemapRule(std::function<void(T1 *, T2 *)> lambda) : lambda(lambda){};
    std::function<void(T1 *, T2 *)> lambda;
    void apply(T1 *in_ptr, T2 *out_ptr) { lambda(in_ptr, out_ptr); };
};

template <class T1 = float, class T2 = float>
struct RangeRemapRule : public RemapRule<T1, T2>
{
    bool invert = false;
    std::pair<T1, T1> from_range;
    std::pair<T2, T2> to_range;
    std::vector<size_t> from_idx;
    std::vector<size_t> to_idx;

    RangeRemapRule(
        const std::pair<T1, T1> &from_range,
        const std::pair<T2, T2> &to_range,
        const std::vector<size_t> &from_idx,
        const std::vector<size_t> &to_idx,
        bool invert) : RemapRule<T1, T2>([from_range, to_range, from_idx, to_idx, invert](T1 *in_ptr, T2 *out_ptr)
                                         {
            for (size_t i = 0; i < from_idx.size(); i++)
            {
                size_t from = from_idx[i];
                size_t to = to_idx[i];
                if (invert)
                    out_ptr[to] = to_range.second + (in_ptr[from] - from_range.first) * (to_range.first - to_range.second) / (from_range.second - from_range.first);
                else
                    out_ptr[to] = to_range.first + (in_ptr[from] - from_range.first) * (to_range.second - to_range.first) / (from_range.second - from_range.first);
            } }){};
};

// template <class T1, class T2>
// struct JointRemapRule : RangeRemapRule<T1, T2>
// {
//     std::string joint = "";
//     ControlType type = CONTROL_EFFORT;

//     JointRemapRule(
//         std::string &joint,
//         std::pair<T1, T1> &from_range,
//         std::pair<T2, T2> &to_range,
//         ControlType type,
//         bool invert) : joint(joint), type(type), RemapRule<T1, T2>(from_range, to_range, invert){};
// };

/** Transformations copy data from any number of buffers to a buffer in specified order.
 *  Should be used whenever data is being copied
 *  Remaps can be
 *
 *  Example:
 *
 *  `
 *  - transform_1:
 *      direction: ToController # It should always be clear which direction data is being transformed
 *      to: to_buffer # Buffer we're copying data to
 *      from:
 *          - buffer_1: # Buffer we're copying data from
 *              from_idx: [0,1,2]
 *              to_idx: [0,1,2]
 *          - buffer_2:
 *              from_idx: [4,2,1,1] # Allow for copying elements in different order as well as repeats
 *              to_idx: [6,5,4,3]
 *              remap: actuator_1
 *  `
 *
 */

template <class T1 = float, class T2 = float>
struct TransformRule
{
    T1 *from_buffer;
    T2 *to_buffer;
    std::vector<RemapRule<T1, T2>> rules;
    TransformRule(T1 *from_buffer, T2 *to_buffer, const std::vector<RemapRule<T1, T2>> &rules) : from_buffer(from_buffer), to_buffer(to_buffer), rules(rules){};
    TransformRule(T1 *from_buffer, T2 *to_buffer, const RemapRule<T1, T2> &rule) : from_buffer(from_buffer), to_buffer(to_buffer)
    {
        rules.emplace_back(rule);
    };

    void apply()
    {
        for (auto &r : rules)
        {
            r.apply(from_buffer, to_buffer);
        }
    }
};