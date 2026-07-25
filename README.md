# gz_betaflight_bridge

This repository contains a standalone C++ bridge between Betaflight SITL and Gazebo Sim Harmonic. The bridge subscribes to Gazebo IMU and altimeter topics, sends Betaflight FDM packets to SITL, receives Betaflight motor commands, converts them to rotor velocity, and publishes `gz.msgs.Actuators` for Gazebo's multicopter motor model.

## Documentation

Start with:

```text
docs/index.md
```

For the main project explanation, requirements, design, plan, and usage, read:

```text
docs/requirements_design_usage.md
```

## Current layout

```text
.
├── CMakeLists.txt
├── CMakePresets.json
├── config/
│   └── bridge.yaml
├── include/
├── README.md
├── docs/
│   └── betaflight_gazebo_bridge_plan.md
├── models/
│   └── betaflight_x3/
├── scripts/
├── src/
│   └── main.cpp
├── test/
├── worlds/
│   └── quadcopter.sdf
└── .vscode/
    ├── extensions.json
    ├── launch.json
    ├── settings.json
    └── tasks.json
```

## Requirements

- CMake 3.23 or newer.
- Ninja.
- A C++20 compiler, such as GCC 13.
- GDB for debugging in VS Code.
- Gazebo Sim Harmonic development packages.
- `yaml-cpp` and `spdlog`.
- VS Code extensions:
  - C/C++ by Microsoft.
  - CMake Tools by Microsoft.

On Ubuntu, install the command-line tools with:

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build gdb libyaml-cpp-dev libspdlog-dev
```

## Configure from the terminal

Configure the debug build with the `debug` CMake preset:

```bash
cmake --preset debug
```

This writes generated build files to:

```text
build/debug
```

The preset also enables:

```text
CMAKE_EXPORT_COMPILE_COMMANDS=ON
```

That produces `build/debug/compile_commands.json`, which VS Code uses for IntelliSense.

## What compile_commands.json is for

`compile_commands.json` tells C++ tools exactly how each source file is compiled.

In this project, the debug preset generates it here:

```text
build/debug/compile_commands.json
```

The file is generated because `CMakePresets.json` sets:

```json
"CMAKE_EXPORT_COMPILE_COMMANDS": "ON"
```

For every `.cpp` file, `compile_commands.json` records the real compiler command CMake uses, including:

- The compiler path, such as `/usr/bin/c++`.
- Include paths.
- Compiler standard flags, such as `-std=gnu++20`.
- Warning flags, such as `-Wall`, `-Wextra`, and `-Werror`.
- Compile definitions.
- The source file path.
- The build directory.

Example shape:

```json
[
  {
    "directory": "/home/user/projects/gz_betaflight_bridge/build/debug",
    "command": "/usr/bin/c++ -std=gnu++20 -Wall -Wextra -Wpedantic -Werror -I/home/user/projects/gz_betaflight_bridge/include -o CMakeFiles/bridge_core.dir/src/BridgeApp.cc.o -c /home/user/projects/gz_betaflight_bridge/src/BridgeApp.cc",
    "file": "/home/user/projects/gz_betaflight_bridge/src/main.cpp"
  }
]
```

VS Code and other C++ language tools use this file to understand the project correctly. It helps with:

- IntelliSense and autocomplete.
- Jump to definition.
- Editor error squiggles.
- Header and include-path resolution.
- `clangd`.
- Static analysis tools.
- Format and lint integrations.

Without `compile_commands.json`, the editor may guess the wrong include paths or compiler flags. That can produce false editor errors even when the project builds correctly.

This workspace points VS Code to the generated file in `.vscode/settings.json`:

```json
"C_Cpp.default.compileCommands": "${workspaceFolder}/build/debug/compile_commands.json"
```

Generate or refresh the file with:

```bash
cmake --preset debug
```

Then build with:

```bash
cmake --build --preset debug
```

Run `cmake --preset debug` again after changing CMake files, adding include directories, adding libraries, or adding new source files.

`compile_commands.json` is generated build output. It should normally not be committed. In this project it is under `build/debug`, and `build/` is ignored by `.gitignore`.

## What CMakePresets.json is for

`CMakePresets.json` stores named CMake configurations in the repository.

Instead of remembering a long command like this:

```bash
cmake -S . -B build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

you can use the preset name:

```bash
cmake --preset debug
```

The preset keeps the build configuration repeatable for the terminal, VS Code, and other developers.

This project currently defines two configure presets:

- `debug`: builds into `build/debug` with `CMAKE_BUILD_TYPE=Debug`.
- `release`: builds into `build/release` with `CMAKE_BUILD_TYPE=Release`.

It also defines matching build presets:

- `debug`: builds the configured debug tree.
- `release`: builds the configured release tree.

The basic terminal workflow is:

```bash
cmake --preset debug
cmake --build --preset debug
```

For release:

```bash
cmake --preset release
cmake --build --preset release
```

The important fields in this project's `CMakePresets.json` are:

```json
{
  "name": "debug",
  "generator": "Ninja",
  "binaryDir": "${sourceDir}/build/debug",
  "cacheVariables": {
    "CMAKE_BUILD_TYPE": "Debug",
    "CMAKE_EXPORT_COMPILE_COMMANDS": "ON"
  }
}
```

Meaning:

- `name` is the preset name used by `cmake --preset debug`.
- `generator` tells CMake to generate Ninja build files.
- `binaryDir` controls where generated build files go.
- `CMAKE_BUILD_TYPE` selects Debug or Release flags.
- `CMAKE_EXPORT_COMPILE_COMMANDS` creates `compile_commands.json` for editor tooling.

## Binding CMake presets to VS Code

VS Code is configured to use the same presets as the terminal.

The binding starts in `.vscode/settings.json`:

```json
"cmake.configureOnOpen": true,
"cmake.useCMakePresets": "always",
"cmake.buildDirectory": "${workspaceFolder}/build/debug",
"C_Cpp.default.compileCommands": "${workspaceFolder}/build/debug/compile_commands.json"
```

Meaning:

- `cmake.configureOnOpen` lets CMake Tools configure the project when the folder opens.
- `cmake.useCMakePresets` tells CMake Tools to use `CMakePresets.json`.
- `cmake.buildDirectory` matches the debug preset output directory.
- `C_Cpp.default.compileCommands` points IntelliSense to the debug preset's compile database.

The build task in `.vscode/tasks.json` also calls the preset directly:

```json
{
  "label": "CMake: build debug",
  "command": "cmake",
  "args": [
    "--build",
    "--preset",
    "debug"
  ]
}
```

That means `Ctrl+Shift+B` runs:

```bash
cmake --build --preset debug
```

The debug configuration in `.vscode/launch.json` uses the debug preset output:

```json
"program": "${workspaceFolder}/build/debug/betaflight_gazebo_bridge",
"preLaunchTask": "CMake: build debug"
```

Meaning:

- VS Code debugs the executable produced by the `debug` preset.
- Before debugging, VS Code runs the debug build task.
- Pressing `F5` rebuilds and then launches GDB on `build/debug/betaflight_gazebo_bridge`.

If VS Code asks which configure preset to use, select:

```text
debug
```

For normal development, use `debug` because it produces better debugger information. Use `release` when you want optimized binaries.

## Build from the terminal

Build the debug executable with:

```bash
cmake --build --preset debug
```

The output executable is:

```text
build/debug/betaflight_gazebo_bridge
```

Run it with:

```bash
./build/debug/betaflight_gazebo_bridge --config config/bridge.yaml
```

Or use:

```bash
scripts/run_bridge.sh
```

## Release build

Configure and build the release preset with:

```bash
cmake --preset release
cmake --build --preset release
```

The release executable is:

```text
build/release/betaflight_gazebo_bridge
```

## Run the Gazebo quadcopter world

This repository includes the Gazebo Sim upstream quadcopter example at:

```text
worlds/quadcopter.sdf
```

It was fetched from:

```text
https://github.com/gazebosim/gz-sim/blob/master/examples/worlds/quadcopter.sdf
```

The world includes the X3 UAV model from Gazebo Fuel and four `gz-sim-multicopter-motor-model-system` plugins. It also loads `gz-sim-physics-system`; without the physics system, Gazebo can receive motor commands but the vehicle will not move.

Before running Gazebo, source the workspace Gazebo environment:

```bash
source scripts/setup_gazebo_env.sh
```

This sets:

```text
GZ_SIM_RESOURCE_PATH
GZ_SIM_SYSTEM_PLUGIN_PATH
```

`GZ_SIM_RESOURCE_PATH` lets Gazebo find local worlds and models from this workspace. `GZ_SIM_SYSTEM_PLUGIN_PATH` lets Gazebo find system plugins built in `build/debug` or `build/release` once this project starts producing Gazebo plugin libraries.

Run the quadcopter world with:

```bash
scripts/run_quadcopter_world.sh
```

Pass normal `gz sim` arguments after the script name. For example, run headless/server-only mode with:

```bash
scripts/run_quadcopter_world.sh -s
```

The upstream world accepts motor velocity commands on:

```text
/X3/gazebo/command/motor_speed
```

Send a simple lift command from another terminal with:

```bash
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
```

Stop the motors with:

```bash
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[0, 0, 0, 0]'
```

## Run Betaflight SITL

The project includes a Betaflight SITL executable at:

```text
bin/betaflight_SITL.elf
```

It was built from the official Betaflight source using:

```bash
make TARGET=SITL
```

The wrapper script is:

```text
scripts/run_betaflight_sitl.sh
```

Show the available SITL options with:

```bash
scripts/run_betaflight_sitl.sh --help
```

Current options:

```text
--ip <address>     Simulator IP address (default: 127.0.0.1)
--config <file>    Load CLI config file, save to EEPROM, and exit
--gpx              Write GPS track to sitl_track.gpx
--help, -h         Show help
```

Run SITL with the default simulator IP:

```bash
scripts/run_betaflight_sitl.sh
```

Run SITL against a specific simulator address:

```bash
scripts/run_betaflight_sitl.sh --ip 127.0.0.1
```

Betaflight SITL is a long-running process. Stop it with `Ctrl+C`.

To configure SITL EEPROM for AUX1 arming and AUX2 ANGLE mode, see:

```text
docs/betaflight_sitl_eeprom.md
```

The CLI file is:

```text
config/betaflight/sitl_modes.cli
```

### See motor traffic on UDP 9002

Betaflight SITL does not send useful motor packets just because the process is running.

The SITL UDP flow is:

```text
Simulator or bridge -> UDP 9003 -> Betaflight SITL
Betaflight SITL -> UDP 9002 -> Simulator or bridge
RC input -> UDP 9004 -> Betaflight SITL
```

Port `9002` is an output port from Betaflight. Betaflight sends a `servo_packet` there after it receives an `fdm_packet` on port `9003`.

The motor packet format is:

```cpp
struct servo_packet {
    float motor_speed[4];
};
```

That is 16 bytes total: four little-endian `float` values.

The FDM packet format is:

```cpp
struct fdm_packet {
    double timestamp;
    double imu_angular_velocity_rpy[3];
    double imu_linear_acceleration_xyz[3];
    double imu_orientation_quat[4];
    double velocity_xyz[3];
    double position_xyz[3];
    double pressure;
};
```

That is 144 bytes total: eighteen little-endian `double` values.

To test this without the Gazebo bridge, open three terminals.

Terminal 1, start SITL:

```bash
scripts/run_betaflight_sitl.sh
```

Terminal 2, listen for motor packets:

```bash
scripts/receive_motors.py
```

Terminal 3, send fake FDM packets:

```bash
scripts/send_test_fdm.py
```

When SITL receives the first FDM packet, it should print something like:

```text
[SITL] new fdm 144 t:...
```

The motor receiver should then print packets from `127.0.0.1`.

At this stage the values may be zero or idle-level because the flight controller is not armed and no RC input is being sent. Seeing packets on `9002` only proves the SITL FDM-to-motor-output loop is alive. To see throttle-dependent motor values later, the bridge must provide FDM continuously and RC/MSP configuration must arm the vehicle and command throttle.

## Full-stack takeoff test

First generate the SITL EEPROM once so AUX1 maps to ARM and AUX2 maps to ANGLE:

```bash
scripts/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```

The easiest way to start Gazebo, Betaflight SITL, the bridge, and MSP hover is:

```bash
scripts/run_msp_hover_stack.sh --headless --target-altitude 5
```

This starts the long-running simulation processes in the correct order and then starts the MSP hover controller after Betaflight's MSP TCP port is ready. It writes logs under:

```text
logs/msp-hover-stack-YYYYMMDD-HHMMSS/
```

Stop everything with `Ctrl+C` in the launcher terminal.

Useful MSP hover stack options:

```bash
scripts/run_msp_hover_stack.sh --target-altitude 5
scripts/run_msp_hover_stack.sh --duration 30 --kp 60 --kd 45 --max-throttle 1650
scripts/run_msp_hover_stack.sh -- --target-altitude 5 --host 127.0.0.1 --port 5761
```

To start only Gazebo, Betaflight SITL, and the bridge:

```bash
scripts/run_takeoff_stack.sh
```

This writes logs under:

```text
logs/takeoff-stack-YYYYMMDD-HHMMSS/
```

Useful legacy options:

```bash
scripts/run_takeoff_stack.sh --headless
scripts/run_takeoff_stack.sh --udp-rc --ramp-end 1600 --hold-duration 20
```

`--udp-rc` is a legacy smoke-test option. The current EEPROM profile uses MSP RC, so use the MSP hover controller for normal hover tests.

Manual flow, if you want separate terminals:

```bash
scripts/run_quadcopter_world.sh
```

```bash
scripts/run_betaflight_sitl.sh
```

```bash
scripts/run_bridge.sh
```

After the bridge logs `imu=true` and `altimeter=true`, run MSP hover:

```bash
scripts/hover_msp_controller.py --target-altitude 5
```

The controller sends low-throttle disarmed RC first, arms at low throttle, then controls throttle using `MSP_ALTITUDE`.

Legacy UDP RC smoke test:

```bash
scripts/run_takeoff_stack.sh --udp-rc --ramp-end 1600
```

More details are in:

```text
docs/takeoff_test.md
```

## MSP hover controller

For hover, use MSP instead of the UDP RC helper. The MSP controller reads Betaflight altitude and sends RC through Betaflight's MSP TCP port.

Regenerate EEPROM once:

```bash
scripts/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```

Start Gazebo, SITL, bridge, and hover together:

```bash
scripts/run_msp_hover_stack.sh --target-altitude 5
```

Or start Gazebo, SITL, and the bridge first:

```bash
scripts/run_takeoff_stack.sh
```

Then run hover in another terminal:

```bash
scripts/hover_msp_controller.py --target-altitude 5
```

Design and usage docs:

```text
docs/msp_hover_controller.md
docs/msp_hover_code_design.md
```

## Configure and build in VS Code

Open this folder in VS Code:

```bash
code .
```

When prompted, install the recommended extensions from `.vscode/extensions.json`.

The workspace is configured to use CMake presets. Select the `debug` configure preset in the CMake Tools status bar if VS Code asks which preset to use.

Build options:

- Press `Ctrl+Shift+B` and choose `CMake: build debug`.
- Or run `Tasks: Run Build Task` from the command palette.
- Or use the CMake Tools `Build` button.

The build task runs:

```bash
cmake --preset debug
cmake --build --preset debug
```

## Debug in VS Code

The debug configuration is `.vscode/launch.json` entry:

```text
Debug betaflight_gazebo_bridge
```

To debug:

1. Open `src/BridgeApp.cc` or another bridge source file.
2. Set a breakpoint in the code path you want to inspect.
3. Open the Run and Debug panel.
4. Select `Debug betaflight_gazebo_bridge`.
5. Press `F5`.

Before launching GDB, VS Code runs the `CMake: build debug` task. That means code changes are rebuilt automatically before each debug session.

## How the files connect

- `CMakeLists.txt` defines the `bridge_core` library, `betaflight_gazebo_bridge` executable, tests, and compiler warnings.
- `CMakePresets.json` defines repeatable `debug` and `release` configure/build presets.
- `.vscode/tasks.json` maps VS Code build tasks to the CMake presets.
- `.vscode/launch.json` starts GDB against `build/debug/betaflight_gazebo_bridge`.
- `.vscode/settings.json` tells VS Code to use CMake presets and the generated compile commands.

## Next project step

The next project step is closed-loop validation: verify the default Betaflight Quad X to Gazebo X3 motor map, then tune `max_rotor_velocity_rad_s` and Gazebo motor constants for stable hover.
