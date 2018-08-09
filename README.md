snap
====

 A controller node for a Kobuki, with straight-forward 'nova' and 'epic' integration, making robot research a snap. 

## Objectives
- Provide autonomous control via service calls (e.g., for 'nova' to utilize).
- Control the robot with a controller/joystick, overriding autonomy.
- Recovery actions if an unexpected bump, floor drop, or wheel drop is detected.

## Dependencies

- ros-<version>-desktop-full
- ros-<version>-kobuki\*
- ros-<version>-ar-track-alvar\*
- ros-<version>-openni\*
- ros-<version>-joy
- ros-<version>-depthimage-to-laserscan
- epic (path planning)
- nova (optional, but ideal)

