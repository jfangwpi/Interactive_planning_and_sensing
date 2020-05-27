# Task Assignment and route-planning among multiple autonomous vehicles system

## 1. Development Environment

* OS: Ubuntu 16.04,
* Compiler: gcc 4.7+
* Building System: CMake

## 2. Install Dependencies

# Update system
```
$ sudo apt-get update
$ sudo apt-get -y upgrade
```

# Development tools for C++, java, and python 
```
$ sudo apt-get -y install build-essential git cmake
$ sudo apt-get -y install openjdk-8-jdk
$ sudo apt-get -y install python-dev python-pip
$ sudo apt-get -y install python-numpy python-scipy python-matplotlib
```

# Commonly used libraries 
```
$ sudo apt-get install autoconf
$ sudo apt-get install libglib2.0-dev
$ sudo apt-get -y install autotools-dev automake autopoint libtool
$ sudo apt-get -y install libopencv-dev python-opencv
$ sudo apt-get -y install libboost-all-dev libeigen3-dev
$ sudo apt-get -y install libcgal-dev
```

# Ceres 
Check http://ceres-solver.org/installation.html#linux to install ceres-solver
```
Clone the ceres solver to folder
$ git clone https://ceres-solver.googlesource.com/ceres-solver 

Install dependency library 
$ sudo apt-get install cmake
$ sudo apt-get install libgoogle-glog-dev
$ sudo apt-get install libatlas-base-dev

Add Eigen to /usr/local/include
$ sudo apt-get install libeigen3-dev

$ sudo apt-get install libsuitesparse-dev
$ sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
$ sudo apt-get update
$ sudo apt-get install libsuitesparse-dev

Install Ceres-solver
Enter the folder
$ cd ~/Software/ceres-solver
$ mkdir ceres-bin
$ cd ceres-bin
$ cmake ../ceres-solver
$ make -j3
$ make test
$ sudo -s
$ make install
```

# Install Visual Studio code (Optional)
Can downloaded from https://code.visualstudio.com/ and installed mannually. Otherwise, run:
```
$ sudo apt-get -y install apt-transport-https 
$ curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
$ sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
$ sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
$ sudo apt-get update
$ sudo apt-get -y install code
```

# LCM (Lightweight Communications and Marshalling)
Check https://lcm-proj.github.io/index.html for more information about LCM
```
Create a folder
$ mkdir -p ~/software/lcm-lib
$ cd ~/software/lcm-lib

Clone LCM source, compile and install
$ git clone https://github.com/lcm-proj/lcm.git lcm
$ cd lcm
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

Post install
$ export LCM_INSTALL_DIR=/usr/local/lib
$ echo $LCM_INSTALL_DIR > /etc/ld.so.conf.d/lcm.conf
```

# Spot (Required by LTL)
Download the latest version of spot from website: https://spot.lrde.epita.fr/install.html
Extract files into ~/software
```
$ cd ~/software
$ ./configure
$ ./configure --disable-python
$ make
$ sudo make install

$ sudo apt-get install gedit
$ gedit ~/.bashrc
Add the following line to your ~/.bashrc
$ export LD_LIBRARY_PATH=:/usr/local/lib/
```


## 3. Set up workspace
Set up the workspace at any location as you prefer. Here I use "~/Workspace/cbba_sim" as example
```
$ mkdir -p ~/Workspace/cbba_sim
$ cd ~/Workspace/cbba_sim
$ git init
$ git remote add origin https://github.com/jfangwpi/task_allocation.git
$ git pull origin master
```

## 4. Build the project
```
$ mdkir build
$ cd build
$ cmake ../src
$ make
```

## 5. Set LCM path (Not required right now)
```
$ gedit ~/.bashrc
Add the follwing lines to the ~/.bashrc
$ export LTLSAMPLING=$HOME/Workspace/ipas
$ export PYTHONPATH=$LTLSAMPLING/src/lcmtypes/python:$LTLSAMPLING/src/lcmtypes/python/communicate_data:$PYTHONPATH
```

## 6. Test the example
### Visualize the map
Edit the configuration files for agents, tasks and map. These configuration files are stored at "/cbba_sim/src/config"
Once the editing is completed, run 
```
$ cd bin
$ ./test_map
```
Check the result at "/ipas/build/bin", the result is the figure called "result_map.jpg". Example of the result can be: 
<img src="/data/result_map.jpg" align="middle" height="500" >
where the actucal mountain environment is 
<img src="/data/mountain_map.jpg" align="middle" height="500" >

#### Comments about result
1. The grey cells are obstacles, orange cells are regions of interest (tasks) and the cell marked by v_i is the initial position of vehicle i.


### Decentralized route-planning for multiple vehicles system in known environments
Once the information of agents, tasks and map is defined, run the following command to visualize the result of task assignment among multiple vehicles by CBBA
```
$ cd bin
$ ./cbba_demo
```
Check the result at "/ipas/build/bin", the result is the figure called "result_cbba.jpg". Example of the result can be: 
<img src="/data/cbba_demo_with_accurate_environment.jpg.jpg" align="middle" height="500" >

#### Comments about result
1. The feasible path for vehicle i is draw by straight line with corresponding color, i.e., vehicle 1 is required to move to cell 6 first, then move to cell 75 along the blue line.  

### Decentralized route-planning for multiple vehicles system in unknown environment - Interactive Planning and Sensing
Once the information of agents, tasks and map is defined, run the following command to visualize the result of task assignment among multiple vehicles by CBBA
```
$ cd bin
$ ./ipad_demo
```
Check the result at "/ipas/src/deomo", the result is the figure called "result_cbba.jpg". Example of the result can be: 
<img src="/data/ipas_grid_map.gif" align="middle" height="500" >