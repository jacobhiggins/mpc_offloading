# mpc_offloading
ROS package for MPC offloading 

# MPC controller for ROS
The code of the MPC controller is contained in the directory `mpc`. The following files are available:

  * `mpc/mpc_shm_ctrl.c` (soon to be renamed as `mpc_ctrl.c`) is the MPC controller in charge of reading the state from and writing the input to a shared memory area. This program may make all the computations or off-load part/all of it to a server
  * `mpc/mpc_ctrl.c` (deprecated) is the old version of the MPC controller which uses a pipe to communicate with ROS
  * `mpc/mpc_server.c` launches a server which listen for client wishing to solve an instance of an MPC problem
  * `mpc/mpc_interface.h` is a C header file which includes the declarations needed to use the MPC controller (such as the shared memory)
  * `mpc/trace_proc.c` is a used to trace the scheduling events of some processes. In the MPC context is used to monitor the schedule of MPC execution, although its usage is not strictly bound to MPC

Below, more detail on each program.

## Preliminaries

Before running the MPC controller, it is necessary to download and compile the MPC library. This step is necessary only the first time of the installation. It can be made by:

1. Clean old compilation stuff by launching the following command from the root project directory
  ```
  make mpc_clean
  ```
This may be necessary to update the MPC library

2. Get the latest MPC library by
  ```
  make mpc_get_lib
  ```

Upon successful completion, you should have got the following files in the `mpc` directory: `mpc.h`, `dyn.h`, `mpc.o`, `dyn.o`. They are needed to compile your MPC controller.

## MPC controller (`mpc_shm_ctrl`)

This is the executable running the local MPC controller. Such a controller interacts with the system to be controlled through a shared memory. The `struct shared_data` declared in `mpc/mpc_interface.h`


2. Compile the MPC controller using the shared memory and used by Matlab, by:
  ```
  make mpc_shm_ctrl
  ```

## Local MPC execution

1. Make sure that USE_SERVER *is not* defined in mpc_ctrl.c. Then
  ```
  make
  ```

## Compilation of a small tracing tool

1. From the home directory of the project, launch
  ```
  make trace_proc
  ```

## Running the MPC Server

1. From project root, run:
```
./launch_MPC_server
```

2. In a new terminal, form project root run:
```
./mpc/mpc_shm_ctrl ./mpc/uav12_iris.json
```

## Compiling ros

From simulator root, run:

```bash
catkin_make
```

This compiles all of the simulator code, including the MPC submodule.

## Launching ROS

** To do: place mpc launch files in folder **

There are two programs to run with ROS.

1. In a terminal, run:
```bash
roslaunch iris_simulator_pk iris_mpc.launch
```

This starts everything that is needed for the simulator. Note: the simulator is NOT included in this repo. This repo serves as a place to put all mpc controller code and is submodule for the ROS simulator.

2. In a terminal, run:
```bash
rosrun mpc_pkg mpc_control
```

This starts the MPC controller.

## Makin the quadrotor move.

1. Select the "keyboard" input window
2. Press 3 to launch
3. Press 1 to move in a house formation