#include "project11_navigation/bt_types.h"

#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "project11_navigation/task.h"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "project11_navigation/plugins/action/multibeam_coverage_action.h"
#include "project11/pid.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <cmath>
#include <ctime>


namespace builtin_interfaces::msg
{

void to_json(nlohmann::json& dest, const Time& time)
{
  dest["seconds"] = time.sec;
  dest["nanoseconds"] = time.nanosec;
  std::time_t total_seconds = static_cast<time_t>(time.sec);
  auto tm = std::gmtime(&total_seconds);
  char timeString[std::size("yyyy-mm-ddThh:mm:ss")];
  std::strftime(std::data(timeString), std::size(timeString), "%FT%T", tm);
  std::stringstream datetime;
  datetime << timeString << "." << std::setw(9) << std::setfill('0') << time.nanosec;
  dest["datetime"] = datetime.str();
}

void from_json(const nlohmann::json& json, Time& time)
{
  json.at("seconds").get_to(time.sec);
  json.at("nanoseconds").get_to(time.nanosec);
}

}

namespace rclcpp
{

void to_json(nlohmann::json& dest, const Time& time)
{
  builtin_interfaces::msg::Time time_msg;
  time_msg = time;
  builtin_interfaces::msg::to_json(dest, time_msg);
}


void from_json(const nlohmann::json& json, Time& time)
{
  builtin_interfaces::msg::Time time_msg;
  builtin_interfaces::msg::from_json(json, time_msg);
  time = time_msg;
}

}


namespace std_msgs::msg
{

void to_json(nlohmann::json& dest, const std_msgs::msg::ColorRGBA& color)
{
  dest["r"] = color.r;
  dest["g"] = color.g;
  dest["b"] = color.b;
  dest["a"] = color.a;
}

void from_json(const nlohmann::json& json, std_msgs::msg::ColorRGBA& color)
{
  json.at("r").get_to(color.r);
  json.at("g").get_to(color.g);
  json.at("b").get_to(color.b);
  json.at("a").get_to(color.a);
}

void to_json(nlohmann::json& dest, const std_msgs::msg::Header& header)
{
  dest["frame_id"] = header.frame_id;
  to_json(dest["stamp"], header.stamp);
}

void from_json(const nlohmann::json& json, std_msgs::msg::Header& header)
{
  json.at("frame_id").get_to(header.frame_id);
  from_json(json.at("stamp"), header.stamp);
}


}

namespace geometry_msgs::msg
{

template<typename T>
void XYZToJson(nlohmann::json& dest, const T& item)
{
  dest["x"] = item.x;
  dest["y"] = item.y;
  dest["z"] = item.z;
}

template<typename T>
void XYZFromJson(const nlohmann::json& json, T& item)
{
  json.at("x").get_to(item.x);
  json.at("y").get_to(item.y);
  json.at("z").get_to(item.z);
}

void to_json(nlohmann::json& dest, const geometry_msgs::msg::Vector3& vector)
{
  XYZToJson(dest, vector);
}

void from_json(const nlohmann::json& json, geometry_msgs::msg::Vector3& vector)
{
  XYZFromJson(json, vector);
}


void to_json(nlohmann::json& dest, const geometry_msgs::msg::Accel& accel)
{
  to_json(dest["linear"], accel.linear);
  to_json(dest["angular"], accel.angular);
}

void from_json(const nlohmann::json& json, geometry_msgs::msg::Accel& accel)
{
  json.at("linear").get_to(accel.linear);
  json.at("angular").get_to(accel.angular);
}

void to_json(nlohmann::json& dest, const geometry_msgs::msg::Point& point)
{
  XYZToJson(dest, point);
}

void from_json(const nlohmann::json& json, geometry_msgs::msg::Point& point)
{
  XYZFromJson(json, point);
}

void to_json(nlohmann::json& dest, const geometry_msgs::msg::Quaternion& quaternion)
{
  XYZToJson(dest, quaternion);
  dest["w"] = quaternion.w;
  double yaw, pitch, roll;
  tf2::getEulerYPR(quaternion, yaw, pitch, roll);
  dest["yaw"] = yaw;
  dest["pitch"] = pitch;
  dest["roll"] = roll;
}

void from_json(const nlohmann::json& json, geometry_msgs::msg::Quaternion& quaternion)
{
  XYZFromJson(json, quaternion);
  json.at("w").get_to(quaternion.w);
}

void to_json(nlohmann::json& dest, const geometry_msgs::msg::Pose& pose)
{
  to_json(dest["position"], pose.position);
  to_json(dest["orientation"], pose.orientation);
}

void from_json(const nlohmann::json& json, geometry_msgs::msg::Pose& pose)
{
  json.at("position").get_to(pose.position);
  json.at("orientation").get_to(pose.orientation);
}


void to_json(nlohmann::json& dest, const geometry_msgs::msg::PoseStamped& pose)
{
  to_json(dest["header"], pose.header);
  to_json(dest["pose"], pose.pose);
}

void from_json(const nlohmann::json& json, geometry_msgs::msg::PoseStamped& pose)
{
  json.at("header").get_to(pose.header);
  json.at("pose").get_to(pose.header);
}

void to_json(nlohmann::json& dest, const geometry_msgs::msg::Twist& twist)
{
  to_json(dest["linear"], twist.linear);
  to_json(dest["angular"], twist.angular);
}

void from_json(const nlohmann::json& json, geometry_msgs::msg::Twist& twist)
{
  json.at("linear").get_to(twist.linear);
  json.at("angular").get_to(twist.angular);
}

void to_json(nlohmann::json& dest, const geometry_msgs::msg::TwistStamped& twist)
{
  to_json(dest["header"], twist.header);
  to_json(dest["twist"], twist.twist);
}

void from_json(const nlohmann::json& json, geometry_msgs::msg::TwistStamped& twist)
{
  json.at("header").get_to(twist.header);
  json.at("twist").get_to(twist.twist);
}

}

namespace project11
{

void to_json(nlohmann::json& dest, const std::shared_ptr<PID>& pid)
{
  if(!pid)
    dest["pid"] = "null";
  else
  {
    dest["Kp"] = pid->Kp();
    dest["Ki"] = pid->Ki();
    dest["Kd"] = pid->Kd();
  }
}

void from_json(const nlohmann::json& json, std::shared_ptr<PID>& pid)
{
}

}

namespace project11_navigation
{

void to_json(nlohmann::json& dest, const std::shared_ptr<Task>& task)
{
  if(!task)
    dest["task"] = "null";
  else
  {
    dest["id"] = task->message().id;
    dest["type"] = task->message().type;
    dest["done"] = task->done();
    dest["status"] = task->message().status;
  }
}

void from_json(const nlohmann::json& json, std::shared_ptr<Task>& task)
{
}


// void to_json(nlohmann::json& dest, const std::shared_ptr<MultibeamCoverageActionClient>& client)
// {
//   if(!client)
//     dest["action_client"] = "null";
//   else
//   {
//     dest["line_count"] = client->lineCount();
//     dest["last_line_number"] = client->lastLineNumber();
//     dest["done"] = client->done();
//     auto sac = client->actionClient();
//     if(sac)
//       dest["simple_action_client"] = "null";
//     else
//     {
//       dest["server_ready"] = sac->action_server_is_ready();
//       //dest["simple_action_client_state"] = sac->getState().getText();
//     }
//   }
// }

// void from_json(const nlohmann::json& json, std::shared_ptr<MultibeamCoverageActionClient>& client)
// {
// }


void registerJsonDefinitions()
{
  BT::RegisterJsonDefinition<std_msgs::msg::ColorRGBA>();
  BT::RegisterJsonDefinition<std_msgs::msg::Header>();
  BT::RegisterJsonDefinition<builtin_interfaces::msg::Time>();
  BT::RegisterJsonDefinition<rclcpp::Time>();

  BT::RegisterJsonDefinition<geometry_msgs::msg::Accel>();
  BT::RegisterJsonDefinition<geometry_msgs::msg::Point>();
  BT::RegisterJsonDefinition<geometry_msgs::msg::Pose>();
  BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
  BT::RegisterJsonDefinition<geometry_msgs::msg::Quaternion>();
  BT::RegisterJsonDefinition<geometry_msgs::msg::Twist>();
  BT::RegisterJsonDefinition<geometry_msgs::msg::TwistStamped>();
  BT::RegisterJsonDefinition<geometry_msgs::msg::Vector3>();

  BT::RegisterJsonDefinition<std::shared_ptr<Task> >();
  // BT::RegisterJsonDefinition<std::shared_ptr<MultibeamCoverageActionClient> >();
  BT::RegisterJsonDefinition<std::shared_ptr<project11::PID> >();
}

} // namespace project11_navigation
