# gz_betaflight_bridge

This repository is being bootstrapped toward a Betaflight SITL to Gazebo Sim bridge. The current first step is a small C++ CMake project that builds and debugs a hello-world executable. Gazebo-specific plugin code will come after the build/debug workflow is working.

## Current layout

```text
.
├── CMakeLists.txt
├── CMakePresets.json
├── README.md
├── docs/
│   └── betaflight_gazebo_bridge_plan.md
├── src/
│   └── main.cpp
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
- VS Code extensions:
  - C/C++ by Microsoft.
  - CMake Tools by Microsoft.

On Ubuntu, install the command-line tools with:

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build gdb
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
    "command": "/usr/bin/c++ -std=gnu++20 -Wall -Wextra -Wpedantic -Werror -o CMakeFiles/hello_bridge.dir/src/main.cpp.o -c /home/user/projects/gz_betaflight_bridge/src/main.cpp",
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
"program": "${workspaceFolder}/build/debug/hello_bridge",
"preLaunchTask": "CMake: build debug"
```

Meaning:

- VS Code debugs the executable produced by the `debug` preset.
- Before debugging, VS Code runs the debug build task.
- Pressing `F5` rebuilds and then launches GDB on `build/debug/hello_bridge`.

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
build/debug/hello_bridge
```

Run it with:

```bash
./build/debug/hello_bridge
```

Expected output:

```text
Hello from gz_betaflight_bridge bootstrap.
```

## Release build

Configure and build the release preset with:

```bash
cmake --preset release
cmake --build --preset release
```

The release executable is:

```text
build/release/hello_bridge
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

The world includes the X3 UAV model from Gazebo Fuel and four `gz-sim-multicopter-motor-model-system` plugins. It is useful as the starting world for the future Betaflight bridge plugin.

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
Debug hello_bridge
```

To debug:

1. Open `src/main.cpp`.
2. Set a breakpoint on the `std::cout` line.
3. Open the Run and Debug panel.
4. Select `Debug hello_bridge`.
5. Press `F5`.

Before launching GDB, VS Code runs the `CMake: build debug` task. That means code changes are rebuilt automatically before each debug session.

## How the files connect

- `CMakeLists.txt` defines the `hello_bridge` executable and compiler warnings.
- `CMakePresets.json` defines repeatable `debug` and `release` configure/build presets.
- `.vscode/tasks.json` maps VS Code build tasks to the CMake presets.
- `.vscode/launch.json` starts GDB against `build/debug/hello_bridge`.
- `.vscode/settings.json` tells VS Code to use CMake presets and the generated compile commands.

## Next project step

The planning document in `docs/betaflight_gazebo_bridge_plan.md` describes Milestone 0 as a buildable Gazebo system plugin with packet layout tests. After this hello-world bootstrap is confirmed, the next implementation step is to replace the executable-only scaffold with the first shared-library plugin target and add verified packet definitions.
