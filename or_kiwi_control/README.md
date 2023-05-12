# Simulink control of the Kiwi SCARA robot

This repo contains the Simulink model and additional code for controlling the Kiwi SCARA robot using the ERT Linux real-time target. It provides the client-side infrastructure for the control laboratory of the Fundamentals of robotics course laboratory at FER.

## System overview

TODO: Provide a figure of the system: client->target->RoboteQ->Kiwi

## Installation

The software has been tested with Matlab R2014a and R2015a on Ubuntu 14.04 and Ubuntu 16.04. This repo provides the client side software. In order to control the robot, the Real-time Linux target machine must be set up as described in (TODO: Mirkovic rad za rektorovu, link na image).

### gcc 4.7

Matlab R2014a requires gcc-4.7, so this version must be installed and set as default system-wide before starting Matlab, in order for Matlab to find it. Another *important* reason why gcc 4.7 is required might have to do with the runtime libraries installed on the target PC. Using a newer gcc version (supported by newer versions of Matlab) might lead to crashes of the executables generated from Simulink models. The `update-alternatives` tool offers a nice way of choosing between gcc versions. In this example (applicable to Ubuntu 16.04) we'll enable choosing between gcc 5 and gcc 4.7.

```
sudo apt install gcc-4.7 g++-4.7
sudo update alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.7 50 --slave /usr/bin/g++ g++ /usr/bin/g++-4.7
sudo update alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 100 --slave /usr/bin/g++ g++ /usr/bin/g++-5
```

This gives a higher priority (in this case 100, which is an arbitrary positive integer) to gcc 5 (the default gcc version on Ubuntu 16.04), but lets us switch to gcc 4.7 using
```
sudo update-alternatives --config gcc
```
which should be invoked every time before starting Matlab.

**Note:** You could also give a higher priority to gcc 4.7 making it the system-wide default, but this might cause problems with other software you might be developing on the same machine, as most projects "expect" the default gcc version.

### ERT Linux

Clone the [ERT Linux repo](https://github.com/larics/ert_linux.git) to a location of your choice (e.g. `~/Documents/MATLAB`) and make sure to check out the [Kiwi branch](https://github.com/larics/ert_linux/tree/kiwi).
```
git clone https://github.com/larics/ert_linux.git
cd ert_linux
git fetch
git checkout -b test origin/test
```
Then open Matlab, navigate to the `ert_linux` folder and run the installation script
```
>> ert_linux_setup
```
which adds the `ert_linux` folder to Matlab's search path.

## Running the controller

Clone this repo to a location of your choice:
```
git clone --recurse-submodules https://github.com/larics/or_kiwi_contol.git
```

Make sure that the system is connected properly. An Ethernet connection should be established between the client PC and the target machine. Start Matlab, `cd` to this folder and run the following steps once:

1. Compile the BCI CAN driver 
```
>> build_can_mex
```
2. Load the control diagram parameters
```
>> parametri
```

Open `Kiwi_control.slx` and perform the following sequence of steps for each experiment
1. Compile the model; this step actually generates C code from the model, compiles it into a binary executable, transfers the executable to the target machine and runs it

2. Connect to the target machine

3. Run the model

4. Stop the model; This automatically disconnects the Simulink diagram from the target machine; If you skip this step, you will not be able to access the recorded data, and you will need to manually kill the `Kiwi_control` process on the target machine before running the next experiment

5. Use the `dohvati.m` function to get the experiment data as a structure:
```
d = dohvati;
```


