# Y-Expanded Grasp Approach Design

## Goal

Before either arm descends toward the box, move the two end-effectors outward along the planning-frame y axis so the approach cannot pass through the box center.

## Design

The grasp waypoint sequence gains one high-clearance waypoint per arm:

`current -> clearanceLift -> yExpanded -> via -> preGrasp -> grasp -> hold -> lift`

For the left arm, `yExpanded.y` is the box center y plus `box_size_y / 2 + approach_box_clearance`; for the right arm it is the corresponding negative offset. The x position, z height, and orientation remain those of the existing high-clearance approach pose. The following `via -> preGrasp` segment therefore starts only after the arms are separated outside the box footprint.

The expansion is computed from the existing box width and clearance parameters; no new fixed distance parameter is introduced. The new segment uses the existing approach timing and trajectory scaling so launch interfaces remain unchanged.

## Validation

- Source-level planner tests verify the y-expanded waypoint is generated and placed outside the box y bounds.
- Existing waypoint validation and grasp/lift tests remain unchanged and must pass.
- The planner target trajectory must have matching waypoint and time-offset counts.
