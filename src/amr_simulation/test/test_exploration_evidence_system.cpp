#include <set>
#include <string>

#include <gtest/gtest.h>

#define AMR_EXPLORATION_EVIDENCE_SYSTEM_TEST
#include "../src/exploration_evidence_system.cpp"

namespace
{

using amr_simulation::AppendDeduplicatedContacts;
using amr_simulation::ContactKey;
using amr_simulation::CoveragePayload;
using amr_simulation::FillCollisionName;

gz::sim::Entity AddNamedEntity(
  gz::sim::EntityComponentManager &ecm,
  const gz::sim::Entity parent,
  const std::string &name)
{
  const auto entity = ecm.CreateEntity();
  ecm.CreateComponent(entity, gz::sim::components::Name(name));
  if (parent != gz::sim::kNullEntity) {
    ecm.CreateComponent(entity, gz::sim::components::ParentEntity(parent));
  }
  return entity;
}

}  // namespace

TEST(ExplorationEvidenceSystemTest, RecoversScopedCollisionNameFromEntityAncestry)
{
  gz::sim::EntityComponentManager ecm;
  const auto model = AddNamedEntity(ecm, gz::sim::kNullEntity, "amr");
  ecm.CreateComponent(model, gz::sim::components::Model());
  const auto link = AddNamedEntity(ecm, model, "base_link");
  ecm.CreateComponent(link, gz::sim::components::Link());
  const auto collision = AddNamedEntity(ecm, link, "base_collision");
  ecm.CreateComponent(collision, gz::sim::components::Collision());

  gz::msgs::Entity message;
  message.set_id(collision);
  FillCollisionName(message, ecm);

  EXPECT_EQ(message.name(), "amr::base_link::base_collision");
  EXPECT_EQ(message.type(), gz::msgs::Entity::COLLISION);
}

TEST(ExplorationEvidenceSystemTest, DeduplicatesMirroredCollisionComponents)
{
  gz::sim::EntityComponentManager ecm;
  const auto first = AddNamedEntity(ecm, gz::sim::kNullEntity, "first_collision");
  const auto second = AddNamedEntity(ecm, gz::sim::kNullEntity, "second_collision");
  ecm.CreateComponent(first, gz::sim::components::Collision());
  ecm.CreateComponent(second, gz::sim::components::Collision());

  gz::msgs::Contacts first_component;
  auto *forward = first_component.add_contact();
  forward->mutable_collision1()->set_id(first);
  forward->mutable_collision2()->set_id(second);
  forward->add_depth(0.01);

  gz::msgs::Contacts second_component;
  auto *reverse = second_component.add_contact();
  reverse->mutable_collision1()->set_id(second);
  reverse->mutable_collision2()->set_id(first);
  reverse->add_depth(0.01);

  gz::msgs::Contacts output;
  std::set<std::string> seen;
  AppendDeduplicatedContacts(first_component, ecm, output, seen);
  AppendDeduplicatedContacts(second_component, ecm, output, seen);

  ASSERT_EQ(output.contact_size(), 1);
  EXPECT_EQ(
    ContactKey(output.contact(0)),
    ContactKey(*forward));
  EXPECT_FALSE(output.contact(0).collision1().name().empty());
  EXPECT_FALSE(output.contact(0).collision2().name().empty());
}

TEST(ExplorationEvidenceSystemTest, CoverageHeartbeatCarriesCountsAndSimulationStamp)
{
  const auto payload = CoveragePayload(
    std::chrono::nanoseconds(1234567890), 7, 6, 2, 11);

  EXPECT_NE(payload.find("\"simulation_time_ns\":1234567890"), std::string::npos);
  EXPECT_NE(payload.find("\"monitored_collisions\":7"), std::string::npos);
  EXPECT_NE(payload.find("\"covered_collisions\":6"), std::string::npos);
  EXPECT_NE(payload.find("\"contact_count\":2"), std::string::npos);
  EXPECT_NE(payload.find("\"command_receipt_count\":11"), std::string::npos);
}
