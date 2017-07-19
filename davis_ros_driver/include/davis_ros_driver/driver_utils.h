// This file is part of DVS-ROS - the RPG DVS ROS Package
//
// DVS-ROS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DVS-ROS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DVS-ROS.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <vector>

namespace davis_ros_driver {

template<typename T>
T clip(T n, T lower, T upper) {
    return std::max(lower, std::min(n, upper));
}

template<typename T>
float mean(const std::vector<T>& v)
{
    if(v.empty())
    {
        return 0.f;
    }

    float sum = static_cast<float>(std::accumulate(v.begin(), v.end(), 0.0));
    float mean = sum / v.size();
    return mean;
}

/* Trimmed mean: removes the first and last proportion_to_cur percentiles of the data
 *  before computing the mean, e.g.:
 *     proportion_to_cut = 0 -> normal mean
 *      proportion_to_cur = 0.5 -> median
 */
template<typename T>
float trim_mean(const std::vector<T>& v_original, const float proportion_to_cut = 0)
{
    if(v_original.empty())
    {
        return 0.f;
    }

    std::vector<T> v(v_original);
    std::sort(v.begin(), v.end());

    const size_t size = v.size();
    const size_t num_values_to_cut = static_cast<int>(size * proportion_to_cut);
    const size_t start_index = num_values_to_cut - 1;
    const size_t end_index = size - num_values_to_cut - 1;

    std::vector<T> trimmed_vec(v.begin() + start_index, v.begin() + end_index);
    return mean(trimmed_vec);
}

} // namespace
