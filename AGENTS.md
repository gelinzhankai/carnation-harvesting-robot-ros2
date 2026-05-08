# AGENTS.md

## Project rules
This is a graduation-design project for a carnation harvesting robot.
The project scope is software design and simulation only.
Do not assume real hardware exists.

## Primary goal
Build a minimum viable ROS 2 simulation pipeline:
robot model -> target pose -> motion planning -> action sequence -> visualization

## Engineering principles
- Prefer minimum working pipeline over full realism
- Keep code modular and readable
- Explain package purpose before generating code
- Use Python unless C++ is clearly necessary
- Add launch files for every module
- Add debug logs and topic comments
- Avoid hidden assumptions

## System assumptions
- simplified carnation scene is allowed
- target objects can be cylinders + spheres
- pixel-to-3D conversion may begin with fixed-depth or fixed-plane assumption
- dual-arm structure: upper arm for hold/stabilize, lower arm for cut

## Task strategy
When asked to implement something:
1. describe architecture
2. propose file tree
3. generate code
4. provide run commands
5. provide debug checklist

## Things to avoid
- do not overengineer
- do not require physical sensors or actuators
- do not introduce unnecessary dependencies
- do not rewrite the whole project if only one module is requested