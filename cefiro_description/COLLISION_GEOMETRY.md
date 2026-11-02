# CEFIRO collision geometry

## Scope

Only collision geometry was added in this revision. The original visual STL
meshes, joint tree, kinematics, inertia values and materials remain unchanged.
There is no mesh-backed `<collision>` element. The collision primitives are
configured with `collision_parts` in `config/cefiro_params.yaml`.

- `base_link`: central pedestal and four diagonally braced feet.
- `narya_ring`: the lower crossbar, compact yaw hub, **one straight diagonal box per
  side** (not two joined at an elbow), and two slim upper uprights. The upper
  uprights stop at Z = 408.827 mm in Narya's frame; the lower Vilya ring
  surface is around Z = 419.827 mm at zero pitch. The uprights are also
  narrowed to 12 mm and shifted slightly outboard to keep clearance near the
  pitch pivot. The center opening remains unobstructed.
- `vilya_ring`: eight thin straight bars approximating the perimeter of the horizontal octagonal ring.
- `nenya_ring`: two lateral ties, small outer pivots and an open rectangular mounting tray.
- `drone_body_link`: central chassis, four diagonal arms and compact motor mounts.
- `prop_1_link` through `prop_4_link`: thin cylinders covering the full rotating
  swept area of each 10-inch rotor; they follow the corresponding CW/CCW joints.

All positions/sizes are in millimeters; rpy values are radians. These are
**engineering collision approximations**, not exact CAD collision solids. The
outer bearing contacts between adjacent gimbal links may overlap intentionally
as real pivots do, and dynamic self-contact filtering should be handled by the
physics engine (especially for adjacent joints). The collision volumes were
reduced to avoid enclosing the mechanism with large solid boxes. Clearance
must be reverified in Gazebo across the actual permissible joint-angle ranges.

## Commands

```bash
ros2 launch cefiro_description display.launch.py enable_collision:=true

# Re-check generated URDF structure and primitives:
xacro $(ros2 pkg prefix cefiro_description)/share/cefiro_description/urdf/cefiro-gyro.urdf.xacro > /tmp/cefiro.urdf
check_urdf /tmp/cefiro.urdf
```

In RViz2: RobotModel > Visual Enabled = false; Collision Enabled = true,
for an inspection of the primitive approximations. Leave Visual Enabled true
and Collision Enabled false for the usual appearance.

Collision geometry does not automatically enforce limits or safe articulation
ranges. The main gimbal joints remain continuous as in the source YAML.

## Collision-only RViz view

Use the included collision-only RViz layout (the original `cefiro.rviz` is untouched):

```bash
ros2 launch cefiro_description display.launch.py \
  rviz_config:=$(ros2 pkg prefix cefiro_description)/share/cefiro_description/rviz/cefiro_collisions.rviz
```

The original `collision_dimensions_mm` fields were retained byte-for-byte for
backwards compatibility; `collision_parts` is now the source of geometry.

A conservative oriented-box clearance check at zero yaw/pitch/roll did not
identify interpenetration between nonadjacent links. Adjacent mechanical pivots
may contact. At appreciable roll angles, the 254-mm rotor swept volumes can
approach or intersect the Vilya ring. This may be a physical safety limit,
not just a modeling artifact; verify the permitted angular travel before any
propeller spin experiment.

## Narya side clearance revision

The prior `lower_diagonal_*` and `upper_diagonal_*` collision boxes were
replaced by `continuous_diagonal_negative` and
`continuous_diagonal_positive` (one straight box on each side). The
visual STLs, joint origins, inertial data and every other link's
collision geometry were left unchanged. Narya now contains 6 instead of
8 collision primitives; the complete robot contains 44.

An oriented-box separation check sampled pitch angles over -180 to +180
degrees: these new side diagonals and uprights do not overlap the Vilya
collision bars in the sampled poses. A pre-existing contact is still
possible between `yaw_lower_hub` and Vilya's `right_flat` near +/-90
degrees pitch; those boxes were not changed in this targeted revision.
This check is a geometric approximation, not a guarantee of physical
clearance, especially for moving parts and spinning rotors.
