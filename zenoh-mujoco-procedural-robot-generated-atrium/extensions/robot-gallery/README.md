# Robot gallery

This bounded robot-generator extension owns 50 structural robot recipes. `robot_catalog.json` lists each family's title, category, and description; the RobotLens Robots panel reads it to show one card per family. Configure an instance's `family` and `scale` fields, then call `robot.generate` over MCP with its instance ID and a stable robot ID. The generated graph is validated and saved as an editable project robot. For the gallery, use one instance per robot so each family keeps independent settings. Explicit `replace=true` regenerates a robot that came from the same instance; ordinary graph edits remain saved until that command is used.

Each generated robot rests on the ground plane: the root link is lifted so the lowest primitive touches z = 0. Cylinders and capsules are oriented along their links, arm and leg joints use pitch, yaw, and roll axes with per-joint limits, and mass and inertia are derived from primitive volume. Collision primitives are slightly smaller than visuals except for wheels, feet, and other contact parts. Wheels use velocity actuators; arm, leg, and gripper joints use position actuators; passive joints have none. Sensors are separate fixed links named `lidar_link`, `camera_link`, and `imu_link`.

All robot settings live in this package's configuration, edited in Window → Extensions or with `extension.configure`; RobotLens has no separate robot property editor. After changing settings, run `robot.generate` with `replace=true`.

| Setting | Effect |
| --- | --- |
| `family`, `scale` | Robot recipe and overall size |
| `variation` | How much the seed changes proportions; 0 disables it |
| `reach` | Arm reach in meters for arm families; 0 uses the family default |
| `limb_length` | Leg length multiplier for legged and humanoid families |
| `segments` | Snake and continuum arm segment count; 0 lets the seed choose |
| `chassis_length`, `chassis_width`, `chassis_height`, `wheel_radius`, `ground_clearance` | Mobile base dimensions in meters; 0 uses the family default |
| `joint_limit_scale`, `effort_scale` | Revolute range and actuator effort multipliers |
| `gripper`, `gripper_opening` | Two-finger gripper and per-finger travel |
| `lidar`, `camera`, `imu` | Include each sensor link |
| `custom_colors`, `body_color`, `accent_color`, `arm_color` | Replace the family colors |
| `x`, `y`, `yaw` | Robot placement on the ground plane in meters and degrees |

With default settings the seed picks the accent color and varies limb lengths and segment counts.

To add a family, add its recipe and a row to `kFamilies` in `src/robot.cpp`, add the key to the `family` options in `robotlens-package.json`, and add an entry to `robot_catalog.json`.

Rebuild after editing `src/robot.cpp` with `robotlens-extension-harness build <this folder>`, then press Reload in Window → Extensions and regenerate the robots with `replace=true`.

The models are structural approximations. Delta linkage closure, soft materials, flight, underwater dynamics, medical safety, and controllers are outside this package.
