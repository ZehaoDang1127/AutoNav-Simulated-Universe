# Ramp-edge referee geometry

`ramp_edge_referee.patch` corrects a proximity false positive in the pinned
simulator. Attempt 64 failed at base pose `(2.713, 3.189, 1.889)` although the
entire referee-padded body lay outside the rotated ramp. Its nearest lateral
corner was 1.000 m from the side edge. The former one-metre monitoring band
classified that nearby body as an edge departure without any intersection.

Score schema 5 checks the two finite ramp-side segments against the same
padded body and dense physical-pose sweep used for tape contact. Actual edge
contact still fails, including rotated corner contact and crossings between
clear endpoints. The longitudinal ends remain open for ramp traversal.
Speed thresholds, no-progress timeout, tape/cone checks, body padding,
waypoint order and finish checks are retained. Navigation receives no new
ground-truth geometry. Earlier run results retain their original schema.

`tests/ros/test_ramp_edge_referee.py` reproduces the recorded false positive
and tests both actual side edges, rotated contact, swept departure, clear deck,
open-end traversal and a robot wholly beside the ramp.
