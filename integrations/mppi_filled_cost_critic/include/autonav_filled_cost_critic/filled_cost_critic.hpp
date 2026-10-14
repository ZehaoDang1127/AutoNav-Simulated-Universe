#ifndef AUTONAV_FILLED_COST_CRITIC__FILLED_COST_CRITIC_HPP_
#define AUTONAV_FILLED_COST_CRITIC__FILLED_COST_CRITIC_HPP_

#include <memory>
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_smac_planner/collision_checker.hpp"

namespace mppi::critics
{
class FilledCostCritic : public CostCritic
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> filled_checker_;
  std::unique_ptr<nav2_smac_planner::GridCollisionChecker> clearance_checker_;
  double rollout_footprint_margin_{0.0};
  double rollout_clearance_cost_{3000.0};
};
}  // namespace mppi::critics
#endif  // AUTONAV_FILLED_COST_CRITIC__FILLED_COST_CRITIC_HPP_
