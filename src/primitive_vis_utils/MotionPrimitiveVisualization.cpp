/*
motion_primitive_visualization
Copyright (C) 2020 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <primitive_vis_utils/MotionPrimitiveVisualization.h>

#include <cpp_utils/vector_utils.h>

namespace vu = vector_utils;

namespace planner {

MotionPrimitiveVisualization::MotionPrimitiveVisualization()
{
  ROS_INFO("[MotionPrimitiveVisualization] By default, visualizing primitives in the world frame. To change, use the setFrameID(frame) function.");
  frame_id_ = "world";

  // define colors
  red_.a = 1;
  red_.r = 1;
  red_.b = 0;
  red_.g = 0;

  blue_.a = 1;
  blue_.r = 0;
  blue_.b = 1;
  blue_.g = 0;

  yellow_.a = 1;
  yellow_.r = 1;
  yellow_.b = 0;
  yellow_.g = 1;

  grey_.a = 1;
  grey_.r = 0.7;
  grey_.b = 0.7;
  grey_.g = 0.7;

  purple_.a = 0.5;
  purple_.r = 0.5;
  purple_.b = 0.5;
  purple_.g = 0.0;

}

MotionPrimitiveVisualization::~MotionPrimitiveVisualization() {}

void MotionPrimitiveVisualization::setFrameID(const std::string& frame)
{
  frame_id_ = frame;
  if (frame != "world") {
    ROS_INFO("[MotionPrimitiveVisualization] Visualizing primitives in the body frame: %s", frame);
  }
}

// ========================================================================== //

void MotionPrimitiveVisualization::visualizePrimitive(const ForwardArcMotionPrimitives& mp, std_msgs::ColorRGBA& color, ros::Publisher& pub)
{
  auto marker = visualizePrimitive(mp, 0, color);
  pub.publish(marker);
}

visualization_msgs::Marker MotionPrimitiveVisualization::visualizePrimitive(
  const ForwardArcMotionPrimitives& mp, int index, std_msgs::ColorRGBA& color)
{
  visualization_msgs::Marker marker;

  marker.header.stamp = ros::Time::now();
  marker.id = index;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;
  marker.color = color;

  std::vector<FlatState> traj_path;

  marker.header.frame_id = frame_id_;
  if (frame_id_ != "world")
  {
    traj_path = mp.samplePathInLocalFrame(0.05, mp.tfinal());
  } else {
    traj_path = mp.samplePath(0.05, mp.tfinal());
  }

  for (unsigned int i = 0; i<traj_path.size(); i++)
    marker.points.push_back(ros_utils::msgs::toPoint(traj_path[i].pos));

  return marker;
}

// Multiple sequential motion  primitives

void MotionPrimitiveVisualization::visualizeMultiPrimitive(const std::deque<ForwardArcMotionPrimitives>& traj, std_msgs::ColorRGBA& color, ros::Publisher& pub)
{
  auto marker = visualizeMultiPrimitive(traj, 0, color);
  pub.publish(marker);
}

visualization_msgs::Marker MotionPrimitiveVisualization::visualizeMultiPrimitive(
  const std::deque<ForwardArcMotionPrimitives>& traj, int index, std_msgs::ColorRGBA& color, visualization_msgs::Marker& endpoint_markers)
{
  visualization_msgs::Marker line_markers;

  line_markers.header.stamp = ros::Time::now();
  line_markers.id = index;
  line_markers.type = visualization_msgs::Marker::LINE_STRIP;
  line_markers.action = visualization_msgs::Marker::ADD;
  line_markers.pose.orientation.x = 0.0;
  line_markers.pose.orientation.y = 0.0;
  line_markers.pose.orientation.z = 0.0;
  line_markers.pose.orientation.w = 1.0;
  line_markers.scale.x = 0.04;
  line_markers.scale.y = 0.04;
  line_markers.scale.z = 0.04;
  line_markers.color = color;

  endpoint_markers.header.stamp = ros::Time::now();
  endpoint_markers.id = index;
  endpoint_markers.type = visualization_msgs::Marker::SPHERE_LIST;
  endpoint_markers.action =visualization_msgs::Marker::ADD;
  endpoint_markers.pose.orientation.x = 0.0;
  endpoint_markers.pose.orientation.y = 0.0;
  endpoint_markers.pose.orientation.z = 0.0;
  endpoint_markers.pose.orientation.w = 1.0;
  endpoint_markers.scale.x = 0.07;
  endpoint_markers.scale.y = 0.07;
  endpoint_markers.scale.z = 0.07;
  endpoint_markers.color = color;

  std::vector<FlatState> traj_path;
  std::vector<Eigen::Vector3d> endpoints;

  line_markers.header.frame_id = frame_id_;
  endpoint_markers.header.frame_id = frame_id_;
  if (frame_id_ != "world")
  {
    for (size_t i = 0; i < traj.size(); i++)
    {
      std::vector<FlatState> path = traj[i].samplePathInLocalFrame(0.05, traj[i].tfinal());
      traj_path.insert(traj_path.end(), path.begin(), path.end());
      endpoints.push_back(path.back().pos);
    }
  } else {
    for (size_t i = 0; i < traj.size(); i++)
    {
      std::vector<FlatState> path = traj[i].samplePath(0.05, traj[i].tfinal());
      traj_path.insert(traj_path.end(), path.begin(), path.end());
      endpoints.push_back(path.back().pos);
    }
  }

  // Add lines to the list_array.
  for (unsigned int i = 0; i<traj_path.size(); i++)
    line_markers.points.push_back(ros_utils::msgs::toPoint(traj_path[i].pos));

  // Add endpoints to the sphere_array.
  for (unsigned int i = 0; i < endpoints.size(); i++)
    endpoint_markers.points.push_back(ros_utils::msgs::toPoint(endpoints[i]));

  return line_markers;
}

visualization_msgs::Marker MotionPrimitiveVisualization::visualizeMultiPrimitive(
  const std::deque<ForwardArcMotionPrimitives>& traj, int index, std_msgs::ColorRGBA& color)
{
  visualization_msgs::Marker line_markers;

  line_markers.header.stamp = ros::Time::now();
  line_markers.id = index;
  line_markers.type = visualization_msgs::Marker::LINE_STRIP;
  line_markers.action = visualization_msgs::Marker::ADD;
  line_markers.pose.orientation.x = 0.0;
  line_markers.pose.orientation.y = 0.0;
  line_markers.pose.orientation.z = 0.0;
  line_markers.pose.orientation.w = 1.0;
  line_markers.scale.x = 0.04;
  line_markers.scale.y = 0.04;
  line_markers.scale.z = 0.04;
  line_markers.color = color;

  std::vector<FlatState> traj_path;

  line_markers.header.frame_id = frame_id_;
  if (frame_id_ != "world")
  {
    for (size_t i = 0; i < traj.size(); i++)
    {
      std::vector<FlatState> path = traj[i].samplePathInLocalFrame(0.05, traj[i].tfinal());
      traj_path.insert(traj_path.end(), path.begin(), path.end());
    }
  } else {
    for (size_t i = 0; i < traj.size(); i++)
    {
      std::vector<FlatState> path = traj[i].samplePath(0.05, traj[i].tfinal());
      traj_path.insert(traj_path.end(), path.begin(), path.end());
    }
  }

  // Add lines to the list_array.
  for (unsigned int i = 0; i<traj_path.size(); i++)
    line_markers.points.push_back(ros_utils::msgs::toPoint(traj_path[i].pos));

  return line_markers;
}

// Primitive libraries
void MotionPrimitiveVisualization::visualizeMPL(const std::vector<ForwardArcMotionPrimitives> &mp_library, std_msgs::ColorRGBA& color, ros::Publisher& pub)
{
  auto marker = visualizeMPL(mp_library, color);
  pub.publish(marker);
}

visualization_msgs::MarkerArray MotionPrimitiveVisualization::visualizeMPL(const std::vector<ForwardArcMotionPrimitives>& mp_library, const std::vector<int> &intent_sample_indicator, const std::vector<int> &ok_indicator)
{
  visualization_msgs::MarkerArray marker_array;
  size_t num_mp = mp_library.size();

  for (size_t i = 0; i < num_mp; i++) {
    if (intent_sample_indicator[i]) {
      if (!ok_indicator[i]) {
        marker_array.markers.push_back(visualizePrimitive(mp_library[i], i, red_));
      }
      else {
        marker_array.markers.push_back(visualizePrimitive(mp_library[i], i));
      }
    }
  }

  return marker_array;
}

visualization_msgs::MarkerArray MotionPrimitiveVisualization::visualizeMPL(const std::vector<ForwardArcMotionPrimitives>& mp_library, const std::vector<double> &cost, const std::vector<int> &ok_indicator)
{
  if (mp_library.size() != cost.size())
    throw std::invalid_argument("[visualizeMPL] size of mp library and cost are not the same");

  // Vary the colors based on the cost value.
  visualization_msgs::MarkerArray marker_array;
  size_t num_mp = mp_library.size();

  // Rescale the cost to be between 1 and 0.
  std::vector<double> cost_rescaled;
  vu::RescaleMinMax(cost, &cost_rescaled);
  std_msgs::ColorRGBA color;

  for (size_t i = 0; i < num_mp; i++) {
    color.r = 1 - (1 - cost_rescaled[i]) * 0.3;
    // color.g = 0;
    // color.b = 1-cost_rescaled[i];
    color.g = cost_rescaled[i];
    color.b = 0;
    color.a = 0.8;
    if (!ok_indicator[i]) {
      marker_array.markers.push_back(visualizePrimitive(mp_library[i], i, red_));
    }
    else {
      marker_array.markers.push_back(visualizePrimitive(mp_library[i], i, color));
    }
  }

  return marker_array;
}

visualization_msgs::MarkerArray MotionPrimitiveVisualization::visualizeMPL(const std::vector<ForwardArcMotionPrimitives> &mp_library, std_msgs::ColorRGBA& color)
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.clear();
  size_t num_mp = mp_library.size();

  for (size_t i = 0; i < num_mp; i++)
    marker_array.markers.push_back(visualizePrimitive(mp_library[i], i, color));

  return marker_array;
}
// ========================================================================== //

void MotionPrimitiveVisualization::visualizePrimitive(const TimeOptimalPrimitives& mp, std_msgs::ColorRGBA& color, ros::Publisher& pub)
{
  auto marker = visualizePrimitive(mp, 0, color);
  pub.publish(marker);
}

visualization_msgs::Marker MotionPrimitiveVisualization::visualizePrimitive(
  const TimeOptimalPrimitives& mp, int index, std_msgs::ColorRGBA& color)
{
  visualization_msgs::Marker marker;

  marker.header.stamp = ros::Time::now();
  marker.id = index;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;
  marker.color = color;

  std::vector<FlatState> traj_path;

  marker.header.frame_id = frame_id_;
  if (frame_id_ != "world")
  {
    traj_path = mp.samplePathInLocalFrame(0.05, mp.tfinal());
  } else {
    traj_path = mp.samplePath(0.05, mp.tfinal());
  }

  for (unsigned int i = 0; i<traj_path.size(); i++)
    marker.points.push_back(ros_utils::msgs::toPoint(traj_path[i].pos));

  return marker;
}

// Multiple sequential motion  primitives

void MotionPrimitiveVisualization::visualizeMultiPrimitive(const std::deque<TimeOptimalPrimitives>& traj, std_msgs::ColorRGBA& color, ros::Publisher& pub)
{
  auto marker = visualizeMultiPrimitive(traj, 0, color);
  pub.publish(marker);
}

visualization_msgs::Marker MotionPrimitiveVisualization::visualizeMultiPrimitive(
  const std::deque<TimeOptimalPrimitives>& traj, int index, std_msgs::ColorRGBA& color, visualization_msgs::Marker& endpoint_markers)
{
  visualization_msgs::Marker line_markers;

  line_markers.header.stamp = ros::Time::now();
  line_markers.id = index;
  line_markers.type = visualization_msgs::Marker::LINE_STRIP;
  line_markers.action = visualization_msgs::Marker::ADD;
  line_markers.scale.x = 0.04;
  line_markers.scale.y = 0.04;
  line_markers.scale.z = 0.04;
  line_markers.color = color;

  endpoint_markers.header.stamp = ros::Time::now();
  endpoint_markers.id = index;
  endpoint_markers.type = visualization_msgs::Marker::SPHERE_LIST;
  endpoint_markers.action =visualization_msgs::Marker::ADD;
  endpoint_markers.scale.x = 0.07;
  endpoint_markers.scale.y = 0.07;
  endpoint_markers.scale.z = 0.07;
  endpoint_markers.color = color;

  std::vector<FlatState> traj_path;
  std::vector<Eigen::Vector3d> endpoints;

  line_markers.header.frame_id = frame_id_;
  endpoint_markers.header.frame_id = frame_id_;
  if (frame_id_ != "world")
  {
    for (size_t i = 0; i < traj.size(); i++)
    {
      std::vector<FlatState> path = traj[i].samplePathInLocalFrame(0.05, traj[i].tfinal());
      traj_path.insert(traj_path.end(), path.begin(), path.end());
      endpoints.push_back(path.back().pos);
    }
  } else {
    for (size_t i = 0; i < traj.size(); i++)
    {
      std::vector<FlatState> path = traj[i].samplePath(0.05, traj[i].tfinal());
      traj_path.insert(traj_path.end(), path.begin(), path.end());
      endpoints.push_back(path.back().pos);
    }
  }

  // Add lines to the list_array.
  for (unsigned int i = 0; i<traj_path.size(); i++)
    line_markers.points.push_back(ros_utils::msgs::toPoint(traj_path[i].pos));

  // Add endpoints to the sphere_array.
  for (unsigned int i = 0; i < endpoints.size(); i++)
    endpoint_markers.points.push_back(ros_utils::msgs::toPoint(endpoints[i]));

  return line_markers;
}

visualization_msgs::Marker MotionPrimitiveVisualization::visualizeMultiPrimitive(
  const std::deque<TimeOptimalPrimitives>& traj, int index, std_msgs::ColorRGBA& color)
{
  visualization_msgs::Marker line_markers;

  line_markers.header.stamp = ros::Time::now();
  line_markers.id = index;
  line_markers.type = visualization_msgs::Marker::LINE_STRIP;
  line_markers.action = visualization_msgs::Marker::ADD;
  line_markers.scale.x = 0.04;
  line_markers.scale.y = 0.04;
  line_markers.scale.z = 0.04;
  line_markers.color = color;

  std::vector<FlatState> traj_path;

  line_markers.header.frame_id = frame_id_;
  if (frame_id_ != "world")
  {
    for (size_t i = 0; i < traj.size(); i++)
    {
      std::vector<FlatState> path = traj[i].samplePathInLocalFrame(0.05, traj[i].tfinal());
      traj_path.insert(traj_path.end(), path.begin(), path.end());
    }
  } else {
    for (size_t i = 0; i < traj.size(); i++)
    {
      std::vector<FlatState> path = traj[i].samplePath(0.05, traj[i].tfinal());
      traj_path.insert(traj_path.end(), path.begin(), path.end());
    }
  }

  // Add lines to the list_array.
  for (unsigned int i = 0; i<traj_path.size(); i++)
    line_markers.points.push_back(ros_utils::msgs::toPoint(traj_path[i].pos));

  return line_markers;
}

// Primitive libraries
void MotionPrimitiveVisualization::visualizeMPL(const std::vector<TimeOptimalPrimitives> &mp_library, std_msgs::ColorRGBA& color, ros::Publisher& pub)
{
  auto marker = visualizeMPL(mp_library, color);
  pub.publish(marker);
}

visualization_msgs::MarkerArray MotionPrimitiveVisualization::visualizeMPL(const std::vector<TimeOptimalPrimitives>& mp_library, const std::vector<int> &intent_sample_indicator, const std::vector<int> &ok_indicator)
{
  visualization_msgs::MarkerArray marker_array;
  size_t num_mp = mp_library.size();

  for (size_t i = 0; i < num_mp; i++) {
    if (intent_sample_indicator[i]) {
      if (!ok_indicator[i]) {
        marker_array.markers.push_back(visualizePrimitive(mp_library[i], i, red_));
      }
      else {
        marker_array.markers.push_back(visualizePrimitive(mp_library[i], i));
      }
    }
  }

  return marker_array;
}

visualization_msgs::MarkerArray MotionPrimitiveVisualization::visualizeMPL(const std::vector<TimeOptimalPrimitives>& mp_library, const std::vector<double> &cost, const std::vector<int> &ok_indicator)
{
  if (mp_library.size() != cost.size())
    throw std::invalid_argument("[visualizeMPL] size of mp library and cost are not the same");

  // Vary the colors based on the cost value.
  visualization_msgs::MarkerArray marker_array;
  size_t num_mp = mp_library.size();

  // Rescale the cost to be between 1 and 0.
  std::vector<double> cost_rescaled;
  vu::RescaleMinMax(cost, &cost_rescaled);
  std_msgs::ColorRGBA color;

  for (size_t i = 0; i < num_mp; i++) {
    color.r = 1 - (1 - cost_rescaled[i]) * 0.3;
    // color.g = 0;
    // color.b = 1-cost_rescaled[i];
    color.g = cost_rescaled[i];
    color.b = 0;
    color.a = 0.8;
    if (!ok_indicator[i]) {
      marker_array.markers.push_back(visualizePrimitive(mp_library[i], i, red_));
    }
    else {
      marker_array.markers.push_back(visualizePrimitive(mp_library[i], i, color));
    }
  }

  return marker_array;
}

visualization_msgs::MarkerArray MotionPrimitiveVisualization::visualizeMPL(const std::vector<TimeOptimalPrimitives> &mp_library, std_msgs::ColorRGBA& color)
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.clear();
  size_t num_mp = mp_library.size();

  for (size_t i = 0; i < num_mp; i++)
    marker_array.markers.push_back(visualizePrimitive(mp_library[i], i, color));

  return marker_array;
}

// ========================================================================== //
void MotionPrimitiveVisualization::clearMarker(ros::Publisher& pub)
{
  // Create the deletion markers

  delete_marker_.header.stamp = ros::Time::now();
  delete_marker_.id = 0;
  delete_marker_.action = visualization_msgs::Marker::DELETEALL;
  delete_marker_.header.frame_id = "world";

  pub.publish(delete_marker_);
}

void MotionPrimitiveVisualization::clearMarkerArray(ros::Publisher& pub)
{
  delete_marker_.header.stamp = ros::Time::now();
  delete_marker_.id = 0;
  delete_marker_.action = visualization_msgs::Marker::DELETEALL;
  delete_marker_.header.frame_id = "world";

  delete_marker_array_.markers.clear();
  for (int i = 0; i < 100; i++)
  {
    delete_marker_.id = i;
    delete_marker_array_.markers.push_back(delete_marker_);
  }

  pub.publish(delete_marker_array_);
}

} // namespace planner
