/*
motion_primitive_visualization
Copyright (C) 2020 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <deque>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>
#include <time_optimal_primitives/TimeOptimalPrimitives.h>

#include <planning_representations/Trajectory.h>
#include <ros_utils/msgs_utils.h>


namespace planner {

class MotionPrimitiveVisualization
{
  public:

    typedef std::shared_ptr<MotionPrimitiveVisualization> Ptr;
    typedef std::shared_ptr<const MotionPrimitiveVisualization> ConstPtr;

    MotionPrimitiveVisualization();
    ~MotionPrimitiveVisualization();

    void setFrameID(const std::string& frame);

    // ===============ForwardArcMotionPrimitives================== //

    // Single primitives
    void visualizePrimitive(const ForwardArcMotionPrimitives& mp, std_msgs::ColorRGBA& color, ros::Publisher& pub);

    visualization_msgs::Marker visualizePrimitive(const ForwardArcMotionPrimitives& mp, int index) { return visualizePrimitive(mp, index, grey_); }

    visualization_msgs::Marker visualizePrimitive(const  ForwardArcMotionPrimitives& mp, int index, std_msgs::ColorRGBA& color);

    // Multiple primitives
    void visualizeMultiPrimitive(
      const std::deque<ForwardArcMotionPrimitives>& traj, std_msgs::ColorRGBA& color, ros::Publisher& pub);

    visualization_msgs::Marker visualizeMultiPrimitive(
      const std::deque<ForwardArcMotionPrimitives>& traj, int index, std_msgs::ColorRGBA& color);

    visualization_msgs::Marker visualizeMultiPrimitive(
      const std::deque<ForwardArcMotionPrimitives>& traj, int index, std_msgs::ColorRGBA& color, visualization_msgs::Marker& endpoint_marker);

    // Primitive Libraries
    void visualizeMPL(const std::vector<ForwardArcMotionPrimitives> &mp_library, std_msgs::ColorRGBA& color, ros::Publisher& pub);

    visualization_msgs::MarkerArray visualizeMPL(const std::vector<ForwardArcMotionPrimitives> &mp_library, std_msgs::ColorRGBA& color);

    visualization_msgs::MarkerArray visualizeMPL(const std::vector<ForwardArcMotionPrimitives> &mp_library, const std::vector<int> &intent_sample_indicator, const std::vector<int> &ok_indicator);

    visualization_msgs::MarkerArray visualizeMPL(const std::vector<ForwardArcMotionPrimitives>& mp_library, const std::vector<double> &cost, const std::vector<int> &ok_indicator);

    // ===============TimeOptimalPrimitives================== //

    // Single primitives
    void visualizePrimitive(const TimeOptimalPrimitives& mp, std_msgs::ColorRGBA& color, ros::Publisher& pub);

    visualization_msgs::Marker visualizePrimitive(const TimeOptimalPrimitives& mp, int index) { return visualizePrimitive(mp, index, grey_); }

    visualization_msgs::Marker visualizePrimitive(const  TimeOptimalPrimitives& mp, int index, std_msgs::ColorRGBA& color);

    // Multiple primitives
    void visualizeMultiPrimitive(
      const std::deque<TimeOptimalPrimitives>& traj, std_msgs::ColorRGBA& color, ros::Publisher& pub);

    visualization_msgs::Marker visualizeMultiPrimitive(
      const std::deque<TimeOptimalPrimitives>& traj, int index, std_msgs::ColorRGBA& color);

    visualization_msgs::Marker visualizeMultiPrimitive(
      const std::deque<TimeOptimalPrimitives>& traj, int index, std_msgs::ColorRGBA& color, visualization_msgs::Marker& endpoint_marker);

    // Primitive Libraries
    void visualizeMPL(const std::vector<TimeOptimalPrimitives> &mp_library, std_msgs::ColorRGBA& color, ros::Publisher& pub);

    visualization_msgs::MarkerArray visualizeMPL(const std::vector<TimeOptimalPrimitives> &mp_library, std_msgs::ColorRGBA& color);

    visualization_msgs::MarkerArray visualizeMPL(const std::vector<TimeOptimalPrimitives> &mp_library, const std::vector<int> &intent_sample_indicator, const std::vector<int> &ok_indicator);

    visualization_msgs::MarkerArray visualizeMPL(const std::vector<TimeOptimalPrimitives>& mp_library, const std::vector<double> &cost, const std::vector<int> &ok_indicator);

    // =================================================================== //

    void clearMarker(ros::Publisher& pub); // clears a single marker.
    void clearMarkerArray(ros::Publisher& pub); // clears a MarkerArray.

    // Colors
    std_msgs::ColorRGBA red_, grey_, yellow_, purple_, blue_;

  private:
    std::string frame_id_;
    visualization_msgs::Marker delete_marker_;
    visualization_msgs::MarkerArray delete_marker_array_;
};

} // namespace planner
