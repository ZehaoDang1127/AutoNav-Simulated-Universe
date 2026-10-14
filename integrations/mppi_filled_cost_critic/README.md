# Filled-body MPPI cost critic

`FilledCostCritic` retains Humble CostCritic's parameters and center-cost
aggregation, but uses the pinned Smac filled-body checker for every rollout
pose. It rejects lethal cell centers throughout the padded polygon and
inscribed costs at the center, matching the planner and BT. The original
outline checker accepts an interior obstacle and can accept an inscribed
center when the outline is clear; the runtime regression reproduces both.

The local map is locked by MPPI while scoring. An occupied-row index is
rebuilt every cycle, including after rolling and clearing. Unknown policy,
collision penalties, safe-alternative handling, and near-goal cost behavior
are retained. The `consider_footprint: false` mode delegates to CostCritic.

`rollout_footprint_margin` optionally penalizes insufficient extra clearance
during prediction and final-control validation; its default is zero. The
full-track profile uses 0.15 m beyond the live 0.10 m body padding and a
`rollout_clearance_cost` of 500 per affected pose (the plugin default is 3000).
Attempt 63's recorded native scoring preferred standing still over a safe
departure at 3000; 500 reverses that preference while preserving hard-body and
braking validation. The planner and BT retain the same live 0.10 m hard margin,
exceeding the unchanged 0.05 m referee margin. Extra clearance encourages separation while
permitting departure when a new observation occupies only that reserve.
Every lethal live-body overlap remains a hard failure. The extra clearance
cost applies at every sampled and final predicted pose.
The native center-inflation shortcut is disabled
because its live-body radius can miss enlarged-body collisions; the occupied
index still skips empty regions. Regressions cover free-center reserve
costs, safe alternatives and departures, and retained live-body rejection.

Bootstrap builds this package after the Smac overlay. The workspace manifest
records every package source file and the installed critic library hash.
Plugin discovery and actual trajectory scoring are tested in
`tests/ros/filled_cost_critic_test.cpp`. Full-track results remain separate.

The aggregation is adapted from the Apache-2.0
[Navigation2 1.1.20 CostCritic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/cost_critic.cpp).
