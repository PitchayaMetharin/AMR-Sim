#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace amr_simulation
{

constexpr char kCommandTopic[] = "/model/amr/cmd_vel";
constexpr char kContactsTopic[] = "/amr/simulation/diagnostics/contacts";
constexpr char kDeliveredCommandTopic[] =
  "/amr/simulation/diagnostics/delivered_cmd_vel";
constexpr char kCoverageTopic[] =
  "/amr/simulation/diagnostics/contact_coverage";

namespace
{

std::string EntityKey(const gz::msgs::Entity &entity)
{
  if (entity.id() != 0U) {
    return "id:" + std::to_string(entity.id());
  }
  if (!entity.name().empty()) {
    return "name:" + entity.name();
  }
  return "unknown";
}

void AddHeaderValue(
  gz::msgs::Header &header,
  const std::string &key,
  const std::string &value)
{
  auto *entry = header.add_data();
  entry->set_key(key);
  entry->add_value(value);
}

}  // namespace

void FillCollisionName(
  gz::msgs::Entity &message,
  const gz::sim::EntityComponentManager &ecm)
{
  const auto entity = static_cast<gz::sim::Entity>(message.id());
  if (entity == gz::sim::kNullEntity || !ecm.HasEntity(entity)) {
    return;
  }
  if (message.name().empty()) {
    message.set_name(gz::sim::scopedName(entity, ecm, "::", false));
  }
  if (
    message.type() == gz::msgs::Entity::NONE &&
    ecm.Component<gz::sim::components::Collision>(entity) != nullptr)
  {
    message.set_type(gz::msgs::Entity::COLLISION);
  }
}

std::string ContactKey(const gz::msgs::Contact &contact)
{
  auto first = EntityKey(contact.collision1());
  auto second = EntityKey(contact.collision2());
  if (second < first) {
    std::swap(first, second);
  }
  if (first == "unknown" && second == "unknown") {
    return first + "|" + second + "|" + contact.SerializeAsString();
  }
  return first + "|" + second;
}

void AppendDeduplicatedContacts(
  const gz::msgs::Contacts &source,
  const gz::sim::EntityComponentManager &ecm,
  gz::msgs::Contacts &output,
  std::set<std::string> &seen)
{
  for (const auto &raw_contact : source.contact()) {
    gz::msgs::Contact contact(raw_contact);
    FillCollisionName(*contact.mutable_collision1(), ecm);
    FillCollisionName(*contact.mutable_collision2(), ecm);
    if (seen.insert(ContactKey(contact)).second) {
      output.add_contact()->CopyFrom(contact);
    }
  }
}

std::string CoveragePayload(
  const std::chrono::steady_clock::duration simulation_time,
  const std::size_t monitored_collisions,
  const std::size_t covered_collisions,
  const std::size_t contact_count,
  const std::uint64_t command_receipt_count)
{
  const auto simulation_time_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(simulation_time).count();
  std::ostringstream stream;
  stream << "{\"simulation_time_ns\":" << simulation_time_ns
         << ",\"monitored_collisions\":" << monitored_collisions
         << ",\"covered_collisions\":" << covered_collisions
         << ",\"contact_count\":" << contact_count
         << ",\"command_receipt_count\":" << command_receipt_count
         << "}";
  return stream.str();
}

class ExplorationEvidenceSystem final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "Exploration evidence system must be attached to a model" << std::endl;
      return;
    }
    model_entity_ = entity;

    const auto topic = [sdf](const char *element, const char *fallback) {
        return sdf ? sdf->Get<std::string>(element, fallback).first : std::string(fallback);
      };
    command_topic_ = topic("command_topic", kCommandTopic);
    contacts_topic_ = topic("contacts_topic", kContactsTopic);
    delivered_command_topic_ = topic(
      "delivered_command_topic", kDeliveredCommandTopic);
    coverage_topic_ = topic("coverage_topic", kCoverageTopic);

    contacts_publisher_ = node_.Advertise<gz::msgs::Contacts>(contacts_topic_);
    delivered_command_publisher_ =
      node_.Advertise<gz::msgs::Twist>(delivered_command_topic_);
    coverage_publisher_ = node_.Advertise<gz::msgs::StringMsg>(coverage_topic_);
    if (!contacts_publisher_ || !delivered_command_publisher_ || !coverage_publisher_) {
      gzerr << "Exploration evidence system could not advertise diagnostic topics"
            << std::endl;
      return;
    }
    if (!node_.Subscribe(command_topic_, &ExplorationEvidenceSystem::OnCommand, this)) {
      gzerr << "Exploration evidence system could not subscribe to ["
            << command_topic_ << "]" << std::endl;
      return;
    }

    DiscoverCollisions(ecm);
    configured_ = true;
    gzmsg << "Exploration evidence system monitoring "
          << monitored_collisions_.size() << " model collisions and ["
          << command_topic_ << "]" << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_) {
      return;
    }
    DiscoverCollisions(ecm);

    std::deque<ReceivedCommand> pending;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      pending.swap(pending_commands_);
    }
    for (auto &received : pending) {
      gz::msgs::Set(received.message.mutable_header()->mutable_stamp(), info.simTime);
      AddHeaderValue(
        *received.message.mutable_header(), "receipt_count",
        std::to_string(received.receipt_count));
      delivered_command_publisher_.Publish(received.message);
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_) {
      return;
    }

    gz::msgs::Contacts contacts;
    gz::msgs::Set(contacts.mutable_header()->mutable_stamp(), info.simTime);
    std::set<std::string> seen;
    std::size_t covered_collisions = 0U;
    for (const auto collision : monitored_collisions_) {
      const auto *data =
        ecm.Component<gz::sim::components::ContactSensorData>(collision);
      if (data == nullptr) {
        continue;
      }
      ++covered_collisions;
      AppendDeduplicatedContacts(data->Data(), ecm, contacts, seen);
    }
    contacts_publisher_.Publish(contacts);

    gz::msgs::StringMsg coverage;
    coverage.set_data(CoveragePayload(
      info.simTime, monitored_collisions_.size(), covered_collisions,
      static_cast<std::size_t>(contacts.contact_size()),
      command_receipt_count_.load(std::memory_order_acquire)));
    coverage_publisher_.Publish(coverage);
  }

private:
  struct ReceivedCommand
  {
    std::uint64_t receipt_count;
    gz::msgs::Twist message;
  };

  void DiscoverCollisions(gz::sim::EntityComponentManager &ecm)
  {
    std::vector<gz::sim::Entity> collisions;
    for (const auto entity : ecm.Descendants(model_entity_)) {
      if (ecm.Component<gz::sim::components::Collision>(entity) == nullptr) {
        continue;
      }
      collisions.push_back(entity);
      if (ecm.Component<gz::sim::components::ContactSensorData>(entity) == nullptr) {
        ecm.CreateComponent(
          entity, gz::sim::components::ContactSensorData(gz::msgs::Contacts()));
      }
    }
    std::sort(collisions.begin(), collisions.end());
    monitored_collisions_ = std::move(collisions);
  }

  void OnCommand(const gz::msgs::Twist &message)
  {
    const auto receipt =
      command_receipt_count_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_commands_.push_back({receipt, message});
  }

  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> monitored_collisions_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher contacts_publisher_;
  gz::transport::Node::Publisher delivered_command_publisher_;
  gz::transport::Node::Publisher coverage_publisher_;
  std::string command_topic_;
  std::string contacts_topic_;
  std::string delivered_command_topic_;
  std::string coverage_topic_;
  std::atomic<std::uint64_t> command_receipt_count_{0U};
  std::mutex command_mutex_;
  std::deque<ReceivedCommand> pending_commands_;
  bool configured_{false};
};

}  // namespace amr_simulation

#ifndef AMR_EXPLORATION_EVIDENCE_SYSTEM_TEST
GZ_ADD_PLUGIN(
  amr_simulation::ExplorationEvidenceSystem,
  gz::sim::System,
  amr_simulation::ExplorationEvidenceSystem::ISystemConfigure,
  amr_simulation::ExplorationEvidenceSystem::ISystemPreUpdate,
  amr_simulation::ExplorationEvidenceSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  amr_simulation::ExplorationEvidenceSystem,
  "amr_simulation::ExplorationEvidenceSystem")
#endif
