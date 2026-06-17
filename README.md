# FFW-SH5 Teleoperation Simulation

Browser-based physics simulation and teleoperation interface for the ROBOTIS AI Worker FFW-SH5 robot.

Built **without** a dedicated robotics simulator (no MuJoCo, Gazebo, or Isaac Sim) — using Three.js for 3D rendering, Rapier.js for physics, and a custom FK/IK engine.

## Features
- MJCF asset pipeline (XML → Three.js scene)
- Custom Forward Kinematics chain solver
- Keyboard / Gamepad teleoperation
- Rapier.js WASM physics (gravity, collision)
- Runs entirely in the browser

## Robot
**ROBOTIS AI Worker FFW-SH5**: 58 DOF humanoid mobile manipulator  
Assets from [robotis_mujoco_menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)

## Stack
- TypeScript + Vite + React
- Three.js r168+
- Rapier.js (WASM)
- Python (asset conversion scripts)

## Status
🚧 Phase 1 — Asset Pipeline & Basic Rendering

## License
MIT
