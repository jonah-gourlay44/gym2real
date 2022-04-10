#pragma once

#include <string>
#include <vector>
#include <functional>


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