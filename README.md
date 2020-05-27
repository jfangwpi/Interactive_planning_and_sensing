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
Edit the configuration files for agents, tasks and map and true_map. These configuration files are stored at "/cbba_sim/src/config"
In this example, we created an example of mountain search and rescue. The actucal mountain map is stored at "/data/mountain_map.jpg":
<img src="/data/mountain_map.jpg" align="middle" height="500" >

To generate the grid map corresponding to the actual moutain map, run 
```
$ cd bin
$ ./vis_map_demo
```
Check the result at "/ipas/build/bin", the result is the figure called "result_map.jpg". Example of the result can be: 
<img src="/data/result_map.jpg" align="middle" height="500" >

#### Comments about result
1. The black cells are obstacles which can not be passed through by vehicles.


### Decentralized route-planning for multiple vehicles system in known environments
4 vehicles in the corners are required to search the trapped hikers in 10 pre-specified locations marked from P3 - P12. Each yellow-colored region of interest is required a single vehicle to visit. With perfect knowledeg of the environment, the decentralized route-planning for the multi-vehicle system can be obtained by running 
```
$ cd bin
$ ./cbba_demo
```
Check the result at "/ipas/build/bin", the result is the figure called "result_cbba.jpg". Example of the result can be: 
<img src="/data/cbba_demo_with_accurate_environment.jpg" align="middle" height="500" >

#### Comments about result
1. The feasible path for vehicle i is draw by straight line with corresponding color. 


### Decentralized route-planning for multiple vehicles system in unknown environment - Interactive Planning and Sensing
Now assume, no accurate environment is given a prioir. Each vehicle in the cornor is equipped with a drone (sensor) which can be used to explore the unknown environment. The routes for sensors to take measurements at specified locations and the optimal routes for vehicles to accomplish the search and rescue mission can be generated as follows: Open two terminals. In one terminal, run  
```
$ cd bin
$ ./ipas_demo
```

In another terminal, run
```
$ cd /ipas/src/demo/
$ python vis_ipas_lcm.py
```

The results at each iteration will be stored at folder "/ipas/src/deomo" automatically. The gif for the whole interactive planning and sensing for the example of mountain search and rescue is: 
<img src="/data/ipas.gif" align="middle" height="500" >

#### Comments about result
1. At each frame, two plots are displayed. The left one is the probabilistic occupancy map and the right one is the information gain for each vertex in the map. 
2. The yellow lines refers the optimal routes for actors to accomplish the search and rescue mission. The blue dash line refers the routes for sensors to explore the unknown environment.

### Decentralized route-planning for multiple vehicles system in unknown environment - Information-Driven Planning and Sensing
As a comparison, information driven planning and sensing algorithm is implmented to recover the whole unknown environment as much as it can and compute the optimal routes for actors to satisfy the given global mission. The results can be generated: Open two terminals. In one terminal, run (need to run python file first)
```
$ cd /ipas/src/demo/
$ python vis_ipas_lcm.py
```

In another terminal, run
```
$ cd bin
$ ./infor_driven_demo
```

The results at each iteration will be stored at folder "/ipas/src/deomo" automatically. The gif for the information driven approach for the example of mountain search and rescue is: 
<img src="/data/info_driven.gif" align="middle" height="500" >

#### Comments about result
1. Much more sensory resources is required compared to the results shown above.
2. Larger total routes cost is generated by the information-driven approach. It is not always true that more information of environment will provide better results. 


### Decentralized route-planning for multiple vehicles system in unknown environment - Interactive Planning and Sensing With Bayesian Optimization
To consider computational efficiency of information gain, Bayesian optimization is implemented to predict a set of sensors locations without evaluating the information gain for each possible sensors locations. The results can be generated: Open three terminals. In the first terminal, run  (need to run python file first)
```
$ cd /ipas/src/demo/
$ python vis_ipas_lcm.py
```

In the second terminal, run 
```
$ cd /ipas/src/demo/
$ python bayesian_opt_ipas_lcm.py
```

In the last terminal, run
```
$ cd bin
$ ./infor_driven_demo
```

The results at each iteration will be stored at folder "/ipas/src/deomo" automatically. The gif for the interactive planning and sensing with Bayesian optimization for the example of mountain search and rescue is: 
<img src="/data/ipas_bayesian.gif" align="middle" height="500" >

#### Comments about result
1. Similar convergence rate is required by using Bayesian optimization.
2. No optimality scarifice is necessary. 
3. Computational burden is reduced. 
