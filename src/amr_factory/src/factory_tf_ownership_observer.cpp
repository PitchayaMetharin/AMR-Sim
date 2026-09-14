#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rmw/types.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace
{

constexpr double kTfMaxAgeSeconds = 1.0;
constexpr double kPublishPeriodSeconds = 0.1;
const char * const kOwnershipTopic = "/amr/factory/tf_ownership";
const char * const kTfTopic = "/tf";

using Clock = std::chrono::steady_clock;
using GidBytes = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

struct EdgeObservation
{
  double received_monotonic = 0.0;
  int64_t source_timestamp = 0;
  std::string gid;
};

double monotonic_seconds()
{
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

std::string frame_name(const std::string & value)
{
  const auto first = value.find_first_not_of('/');
  if (first == std::string::npos) {
    return {};
  }
  return value.substr(first);
}

std::string full_node_name(const std::string & name, const std::string & node_namespace)
{
  if (!name.empty() && name.front() == '/') {
    return name;
  }
  const std::string normalized_namespace = frame_name(node_namespace);
  if (normalized_namespace.empty()) {
    return "/" + name;
  }
  return "/" + normalized_namespace + "/" + name;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream escaped;
  escaped << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (character < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<unsigned int>(character) << std::dec;
        } else {
          escaped << character;
        }
        break;
    }
  }
  escaped << '"';
  return escaped.str();
}

class TfOwnershipObserver final : public rclcpp::Node
{
public:
  TfOwnershipObserver()
  : Node("tf_ownership_observer")
  {
    const auto ownership_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .transient_local();
    ownership_publisher_ = create_publisher<std_msgs::msg::String>(
      kOwnershipTopic, ownership_qos);

    // TF broadcasters use the sensor-data QoS family.  A best-effort reader
    // remains compatible with both best-effort and reliable publishers.
    const auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      kTfTopic,
      tf_qos,
      std::bind(
        &TfOwnershipObserver::tf_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(kPublishPeriodSeconds)),
      std::bind(&TfOwnershipObserver::publish_evidence, this));
    publish_evidence();
  }

private:
  using EdgeOwners = std::map<std::string, EdgeObservation>;

  static bool same_gid(const rmw_gid_t & message_gid, const GidBytes & endpoint_gid)
  {
    if (message_gid.implementation_identifier == nullptr) {
      return false;
    }
    return std::equal(endpoint_gid.begin(), endpoint_gid.end(), message_gid.data);
  }

  static std::string gid_text(const rmw_gid_t & gid)
  {
    std::ostringstream text;
    text << std::hex << std::setfill('0');
    for (std::size_t index = 0; index < RMW_GID_STORAGE_SIZE; ++index) {
      text << std::setw(2) << static_cast<unsigned int>(gid.data[index]);
    }
    return text.str();
  }

  std::string owner_for_gid(const rmw_gid_t & message_gid) const
  {
    if (message_gid.implementation_identifier == nullptr) {
      return "<unresolved-publisher>";
    }

    std::vector<rclcpp::TopicEndpointInfo> matches;
    try {
      const auto endpoints = get_publishers_info_by_topic(kTfTopic);
      for (const auto & endpoint : endpoints) {
        if (same_gid(message_gid, endpoint.endpoint_gid())) {
          matches.push_back(endpoint);
        }
      }
    } catch (const std::exception &) {
      return "<unresolved-publisher>";
    }

    if (matches.size() != 1) {
      return "<unresolved-publisher>";
    }
    return full_node_name(matches.front().node_name(), matches.front().node_namespace());
  }

  void tf_callback(
    const tf2_msgs::msg::TFMessage::ConstSharedPtr message,
    const rclcpp::MessageInfo & message_info)
  {
    if (!message) {
      return;
    }
    const auto & rmw_info = message_info.get_rmw_message_info();
    const double received_at = monotonic_seconds();
    const std::string owner = owner_for_gid(rmw_info.publisher_gid);
    const std::string gid = gid_text(rmw_info.publisher_gid);

    for (const auto & transform : message->transforms) {
      const std::string parent = frame_name(transform.header.frame_id);
      const std::string child = frame_name(transform.child_frame_id);
      std::string edge;
      if (parent == "map" && child == "odom") {
        edge = "map->odom";
      } else if (parent == "odom" && child == "base_footprint") {
        edge = "odom->base_footprint";
      } else {
        continue;
      }
      observations_[edge][owner] = EdgeObservation{
        received_at,
        rmw_info.source_timestamp,
        gid,
      };
    }
    publish_evidence();
  }

  static void prune(EdgeOwners & owners, double now)
  {
    for (auto iterator = owners.begin(); iterator != owners.end();) {
      if (now - iterator->second.received_monotonic > kTfMaxAgeSeconds) {
        iterator = owners.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  static void append_edge_json(
    std::ostringstream & output,
    const std::string & edge,
    const EdgeOwners & owners)
  {
    output << json_escape(edge) << ":{\"owners\":[";
    bool first = true;
    for (const auto & item : owners) {
      if (!first) {
        output << ',';
      }
      first = false;
      output << json_escape(item.first);
    }
    output << "]}";
  }

  void publish_evidence()
  {
    const double now = monotonic_seconds();
    prune(observations_["map->odom"], now);
    prune(observations_["odom->base_footprint"], now);

    std::ostringstream output;
    output << std::setprecision(17);
    output << "{\"schema\":1,\"kind\":\"factory_tf_ownership\","
           << "\"observed_monotonic\":" << now << ",\"edges\":{";
    append_edge_json(output, "map->odom", observations_["map->odom"]);
    output << ',';
    append_edge_json(
      output, "odom->base_footprint", observations_["odom->base_footprint"]);
    output << "}}";

    std_msgs::msg::String evidence;
    evidence.data = output.str();
    ownership_publisher_->publish(evidence);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ownership_publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::map<std::string, EdgeOwners> observations_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfOwnershipObserver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
