# AMBF Trace Plugin

A plugin for the Asynchronous Multibody Framework (AMBF), see https://github.com/WPI-AIM/ambf
The plugin allows static traces at init or body traces during runtime in the AMBF simulator environment

Developed by Henry Phalen

This was something I made as a convenience tool for my own work, but thought it might be useful to others, so I am putting it here.
There are several features that still could be added (expressed in code TODOs), but it is functional and (I hope) useful as is.

## Features:

1. Show static trace at simulator startup (example screenshot at startup)

![Screenshot 2022-07-29 12:54:50](https://user-images.githubusercontent.com/17507145/181807786-1c9732aa-82b2-4b81-a653-d35ea707df34.png)

2. Dynamically and togglably trace the position of a body. This example video shows toggling collection on and off and showing and hiding the trace

https://user-images.githubusercontent.com/17507145/181808787-0f53f7b9-6a76-4ad7-938b-fe3041fa562a.mp4


## Building
Find most updated instructions for building AMBF on that git repository (https://github.com/WPI-AIM/ambf)

This can be built stand-alone or in a catkin workspace. Instructions below are for a stand-alone build

```git clone``` this repository ```<plugin_path>``` onto your machine

```bash
cd <plugin_path>
mkdir build && cd build
cmake ..
make
```
## Using plugin

### Starting plugin
Find most updated instructions for starting AMBF with plugins on that git repository (https://github.com/WPI-AIM/ambf)

You can test this plugin on the example by:
```<ambf_exe_dir> ---> e.g. ~/ambf/bin/lin-x86_64```

Test body trace:
```bash
cd <ambf_exe_dir>
./ambf_simulator --plugins <plugin_path>/build/libambf_trace_plugin.so --name_body_to_trace Chassis
```

Test static trace:
./ambf_simulator --plugins <plugin_path>/build/libambf_trace_plugin.so --csv_filename_static_traces <plugin_path>/example_static_trace.csv

### Input args
1. ```--name_body_to_trace arg``` : Name of body in ADF yaml which will have position traced. No body trace if blank (default)
2. ```--csv_filename_static_traces arg``` : Path to CSV file with 3d points for static trace. No static trace if blank (default)

### Keyboard commands
(KP means keypad)
1. ```[Ctrl + KP*]``` : start/stop tracing specified body
2. ```[Ctrl + KP-]``` : show/hide body traces

### Starting from launch file
A launch.yaml file can by used to set many simulator parameters at once (world configs, input devices, multibody configs, etc.). You can also add plugins using the following, which would be placed in some ```<launch_file.yaml>```
```bash 
plugins: [
  {
    name: TRACE_PLUGIN,
    filename: libambf_trace_plugin.so,
    path: <plugin_path>/build/
  }
]
```
Then you can start the simulator using:
```bash
./ambf_simulator --launch_file <launch_file.yaml> [<-- plugin_args>]
```
