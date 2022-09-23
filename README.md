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

This plugin can be built stand-alone or in a catkin workspace. 

### Stand-Alone build

```git clone``` this repository ```<plugin_path>``` onto your machine

```bash
cd <plugin_path>
mkdir build && cd build
cmake .. -DBUILD_PLUGIN_WITH_ROS=False
make
```

### Build within a catkin workspace (ROS1)
A simple ```package.xml``` has been included in this repository to enable catkin to find and build it
```<plugin_path>``` should be located within ```catkin_ws/src/```
```
cd <catkin_ws>
catkin build ambf_trace_plugin
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
```bash
cd <ambf_exe_dir>
./ambf_simulator --plugins <plugin_path>/build/libambf_trace_plugin.so --csv_filename_static_traces <plugin_path>/example_static_trace.csv
```

### Input args
1. ```--name_body_to_trace arg``` : Name of body in ADF yaml which will have position traced. No body trace if blank (default)
2. ```--csv_filename_static_traces arg``` : Path to CSV file with 3d points for static trace. No static trace if blank (default)
2. ```--static_trace_rel_body_name arg``` : Name of body given in ADF yaml for static trace to be relative to. Relative to world origin if blank (default)
### Keyboard commands
(KP means keypad)
1. ```[Ctrl + KP*]``` : start/stop tracing specified body
2. ```[Ctrl + KP-]``` : show/hide body traces
2. ```[Ctrl + KP0]``` : reload static trace from file

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

NOTE if you built this plugin using catkin at location ```<catkin_ws>/src/ambf_trace_plugin```, the path will be
```bash
path: ../../build/ambf_trace_plugin/
```

### Features if built with ROS
For now, these features are included based on whether a compiler flag is set at build time. I think there is likely a cleaner way to do that, but unless/until things get long, this should not be an issue. 

If built with ROS, the functionality to set the body trace to [show / hide] and [collect / don't collect] are accessible with ROS topics, in addition to the keyboard press

By default, these topics are at ``` /ambf/trace_plugin/set_body_trace_collect ``` and ```/ambf/trace_plugin/set_body_trace_visible``` respectively, and accept messages of type ```std_msgs::Bool```

## Features to add
Something I hope to get to someday (or you could do it too! :) )

1. Allow passing in a list of bodies, for multiple traces
2. Allow specification of color, line_width
3. Make option for default on/off
4. Allow for multiple static traces
5. Have show/hide functionality also apply to static trace, or make both toggleable separately
6. Allow for throttling of body trace (only happens every x frames)
