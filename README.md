![simulation-demo](image/simulation.gif)

# Fast-Capture

![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange?logo=ubuntu)
![Group](https://img.shields.io/badge/Group-SWUST--ICAA-success)
![License](https://img.shields.io/badge/License-MIT-yellow)

Multi-UAV Cooperative Target Estimation and Capture in Dense Environments

**Video Links**: [simulation experiment](https://youtu.be/oEZhN81ul88),  [indoor experiment](https://youtu.be/27hENS5enrw) or [outdoor experiment](https://youtu.be/twI92W5BkV0)

---

## Build and Install

### Dependencies

- Ubuntu 20.04
- ROS 1

## 1. Prepare
Our software is developed and tested in Ubuntu 20.04. It is recommended to use Yu Xiang shredded pork to install ROS1.

```
wget http://fishros.com/install -O fishros && . fishros
```

### Step 1

Install various dependencies

```
sudo apt-get install libarmadillo-dev
sudo apt-get install libompl-dev
sudo apt-get install gfortran
sudo apt-get install doxygen
```

### Step 2

Get a copy of **MA27** from the [HSL Archive](http://www.hsl.rl.ac.uk/download/MA27/1.0.0/a/). Just select the **Personal Licence (allows use without redistribution)**, then fill the information table. 

​Then you can download it from an e-mail sent to you. Next, un-zip **MA27**, and follow the *README* in it, install it to your Ubuntu.

**Actually, you only need to type 3 commands in MA27's folder to finish the installation.**

```
./configure
make
sudo make install
```

Manually un-zip packages *OOQP.zip* in the repo and install it to your Ubuntu.

**As above, you can just type 3 commands in OOQP's folder :**

```
./configure
make 
sudo make install
```

### Step 3

Download Boost 1.65.1 here [Boost 1.65.1](https://archives.boost.io/release/1.65.1/source/), and after extracting, enter the folder:

```
cd boost_1_65_1
./bootstrap.sh --prefix=/usr/local
sudo ./b2 install
```

Verify Installation:

```
ls /usr/local/lib | grep libboost_system
```

### Step 4

You can create an empty new workspace and clone this repository to your workspace: 

```
cd ~/your_catkin_ws/src
git clone https://github.com/SWUST-ICAA/Fast-Capture.git
cd ..
```
Then, compile it.

```
catkin_make
```

### Step 5

```
source devel/setup.bash
./roundup_simulation
```
Then you can follow the gif below to enjoy it.