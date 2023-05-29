# PC Setup using:
- Ubuntu 20.04
- ROS Neotic

## 1. Install ROS on Remote PC

Open the terminal and type the commands listed below one at a time.

```
sudo apt update
sudo apt upgrade
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/ca11930e-3825-4b7a-a1f8-6524d8d995d5)


```
chmod 755 ./install_ros_noetic.sh 
bash ./install_ros_noetic.sh
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/48ca6e28-b61e-4c25-8235-535b5ddcc4a3)


## 2. Install Dependent ROS Packages

Open the terminal with Ctrl+Alt+T from Remote PC.

```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino ros-noetic-rosserial-python ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/7f1404ab-79af-41e9-aaff-1135fecead67)


## 3 Install TurtleBot3 Packages

Download the source codes and build them.

```
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/0624f6c7-7808-472f-9b3a-257d4530b84b)

![image](https://github.com/Bijay3308/VIP/assets/58104378/8edf0ed2-c114-49de-b0ee-c3eb88a6c2d0)
cd
## 4. Network Configuration

Connect PC to a WiFi device and find the assigned IP address with the command below.

```
ifconfig
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/babf977a-ffb5-4661-895d-dd5320e96928)

Open the file and update the ROS IP settings with the command below.

```
nano ~/.bashrc
```

Press Ctrl+END or Alt+/ to move the cursor to the end of line.
Modify the address of localhost in the ROS_MASTER_URI and ROS_HOSTNAME with the IP address acquired from the above terminal window.

![image](https://github.com/Bijay3308/VIP/assets/58104378/3019934b-3a65-4da1-b6e4-c35bfe2b8b45)


Source the bashrc with below command.

```
source ~/.bashrc
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/9a13f397-abab-4ee4-ad4a-2edf192ebea6)


# SBC Setup using:
- Pen drive
- TurtleBot3 SBC Image (Raspberry Pi 4B (2GB or 4GB) ROS Neotic image)
- Raspberry Pi Imager

Following the preparation of a pen drive, downloading the correct image file for my hardware and the ROS version that is neotic, unzipping the downloaded image file, extracting the.img file, saving it to the local disk and burning the image file using a Raspberry Pi Imager was done. Then the following includes:

```
Click CHOOSE OS.
Click Use custom and select the extracted .img file from local disk.
Click CHOOSE STORAGE and select the microSD.
Click WRITE to start burning the image.
```

![image](https://github.com/sanjiblama28/San/blob/main/Veed%20Recording%20-%206%20November%202022%20(1).gif)

GParted GUI utility must first be installed using the provided code.

```
sudo apt-get install gparted
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/6c48e143-da07-4a68-8bd3-083db44b2d2d)


Next, gparted is launched. 

```
Select microSD card from the menu (mounted location may vary by system).
Right click on the yellow partition.
Select Resize/Move option.
Drag the right edge of the partition to all the way to the right end.
Click Resize/Move button.
Click the Apply All Operations green check button at the top.
```

![image](https://github.com/sanjiblama28/San/blob/main/Veed%20Recording%20-%206%20November%202022%20(3).gif)

## Configure the WiFi Network Setting

Launch a terminal window, and navigate to the netplan directory on the microSD card and open the 50-cloud-init.yaml file and begin editing it with a superuser permission sudo.

```
cd /media/$USER/writable/etc/netplan
sudo nano 50-cloud-init.yaml
```

Replace the WIFI SSID and WIFI PASSWORD with your wifi SSID and password once the editor has been opened.

Press Ctrl+S to save the document and Ctrl+X to close it.

![image](https://github.com/sanjiblama28/San/blob/main/Veed%20Recording%20-%206%20November%202022%20(4).gif)

## Boot Up the Raspberry Pi

```
a. Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.
b. Connect input devices to the USB port of Raspberry Pi.
c. Insert the microSD card.
d. Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.
e. Login with ID ubuntu and PASSWORD turtlebot.
```

## ROS Network Configuration

Please follow the instructions below on the SBC (Raspberry Pi).

Confirm the WiFi IP address.
```
ifconfig
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/175af514-f5ac-4166-bcc8-7192d726bd37)


Edit the .bashrc file.
```
nano ~/.bashrc
```

Find the ROS_MASTER_URI and ROS_HOSTNAME setting section, then modify the IP adddresses accordingly.
```
export ROS_MASTER_URI=http://{IP_ADDRESS_OF_REMOTE_PC}:11311
export ROS_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI_3}
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/c697c866-3524-4a4b-80e1-5c84e05324c9)

Save the file with Ctrl + S and exit the nano editor with Ctrl + X.

Apply changes with the command below.
```
source ~/.bashrc
```

![image](https://github.com/sanjiblama28/VIP/assets/92040822/d5d6e177-48a2-49ea-966a-cb86c9c07dab)


## NEW LDS-02 Configuration

Please follow the steps below on the TurtleBot3 SBC (Raspberry Pi).

1. Install the LDS-02 driver and update the TurtleBot3 package.

```
sudo apt update
sudo apt install libudev-dev
cd ~/catkin_ws/src
git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/catkin_ws/src/turtlebot3 && git pull
rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
cd ~/catkin_ws && catkin_make
```

![image](https://github.com/sanjiblama28/VIP/assets/92040822/bd1f3ccd-db07-4cb6-afb2-0387d7a18d68)

![image](https://github.com/sanjiblama28/VIP/assets/92040822/82f920ef-f76e-471b-b607-2eaa05415958)

Now, 
2. Export the LDS MODEL to the bashrc file. LDS-01 or LDS-02, depending on your LDS model.

```
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
source ~/.bashrc
```
![image](https://user-images.githubusercontent.com/92040822/204987255-ab70dec1-7135-4448-a9c7-7a7fd64bd49a.png)

# OpenCR Setup

1. Connect the OpenCR to the Rasbperry Pi using the micro USB cable.
2. Install required packages on the Raspberry Pi to upload the OpenCR firmware.

```
sudo dpkg --add-architecture armhf
sudo apt update
sudo apt install libc6:armhf 
```

![image](https://user-images.githubusercontent.com/92040822/204988172-e33c0099-0d12-4723-a9ba-ecddeb6efbb6.png)

3. Depending on the platform, use either burger or waffle for the OPENCR_MODEL name.

```
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
```

![image](https://user-images.githubusercontent.com/92040822/204989231-5613bc9d-d103-4406-a464-6ec586d6ce74.png)

4. Download the firmware and loader, then extract the file.

```
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
tar -xvf ./opencr_update.tar.bz2
```

![image](https://user-images.githubusercontent.com/92040822/204989421-5b95e2be-012a-4d2d-a585-8e5998686761.png)

5. Upload firmware to the OpenCR.

```
cd ~/opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

![image](https://user-images.githubusercontent.com/92040822/204989511-07ced6dd-7abe-431b-8127-651a0c2ef6c3.png)


6. OpenCR Test

We perform the OpenCR Test by following the below steps:
--> PUSH SW1 / PUSH SW2 were used to check if the robot has been properly assembeled or not and this process tests the left and right DYNAMIXEL's and the OpenCR board.
--> After assembling TurtleBot3, we connected the power to OpenCR and turn on the power switch of OpenCR. The red `power LED` will be turned on. 
--> And then the robot was placed in a flat ground in an open area.
--> After that, `PUSH SW 1` button was pushed for a few seconds to move the robot forward around 30 centimeters as recommended. 
--> And `PUSH SW 2` for a few seconds to command the robot to rotate 180 degrees in place.

# Bringup

## Run roscore

Run roscore from PC.

```
roscore
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/a190cc4d-a567-481d-8877-7a981083fc0a)

# Bringup TurtleBot3

TIP: Before executing this command, you have to specify the model name of TurtleBot3. The ${TB3_MODEL} is the name of the model you are using in burger, waffle, waffle_pi.

Open a new terminal from PC with Ctrl + Alt + T and connect to Raspberry Pi with its IP address.
The default password is turtlebot.

```
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/3712576a-11ca-4167-976b-af151dbcc50c)

Bring up basic packages to start TurtleBot3 applications.

```
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/5f3eae68-3db2-45c9-becf-1e316606e502)

```
We were unable to remedy this issue despite the fact that neither the Remote PC's IP nor the Raspberry Pi's IP had a problem, and that both were connected to the same network, but the first three segments of their IP addresses were different. i.e. IP od Remote PC = 192.168.182.131 and IP of Raspberry Pi = 192.168.0.91 I attempted to resolve the problem by checking network connectivity, firewall and security settings, and ROS environment variables, as well as verifying ROS node network configuration and restarting the ROS core, but I was unable to find a solution.
```

# Simulation of Obstacle Detection

## Launch Simulation World

In this instruction, TurtleBot3 World will be used.
Please use the proper keyword among burger, waffle, waffle_pi for the TURTLEBOT3_MODEL parameter.

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/a7717f91-e9e2-4c55-8197-5d8b635600d2)

## Run SLAM Node

Open a new terminal from Remote PC with Ctrl + Alt + T and run the SLAM node. Gmapping SLAM method is used by default.
Please use the proper keyword among burger, waffle, waffle_pi for the TURTLEBOT3_MODEL parameter.

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

![image](https://github.com/Bijay3308/VIP/assets/58104378/27360d7f-81f2-4695-a59e-6481a294662a)

## Run Teleoperation Node

Open a new terminal from Remote PC with Ctrl + Alt + T and run the teleoperation node from the Remote PC.
Please use the proper keyword among burger, waffle, waffle_pi for the TURTLEBOT3_MODEL parameter.

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
 ```
 
![image](https://github.com/Bijay3308/VIP/assets/58104378/16e91b52-caf3-4a4a-9dd1-8da843a4bbb2)

 ## Run Obstacle launch
 
 ```
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_example turtlebot3_obstacle.launch
 ```
 
![image](https://github.com/Bijay3308/VIP/assets/58104378/be72de1b-4238-4572-b029-6e8046aa748d)

 # Simulation of Pick-up and Drop
 
## Run Gazebo

[Remote PC] Load TurtleBot3 with OpenMANIPULATOR-X into Gazebo world with the following command.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```

![image](https://github.com/sanjiblama28/VIP/assets/92040822/7a46cdf2-9327-4a30-8ad5-d159f0e740fc)

## Run move_group Node

[Remote PC] In order to use Moveit feature, launch move_group node. If you press [▶] button in Gazebo to start simulation, use the following command.
With a successful launch, “You can start planning now!” message will be printed on the terminal.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```

![image](https://github.com/sanjiblama28/VIP/assets/92040822/8723f78e-c760-42be-a544-6ad46103d996)

![image](https://github.com/sanjiblama28/VIP/assets/92040822/d54d2740-cc99-40a6-aa82-853e0ec36149)

## Run RViz

[Remote PC] Use Moveit feature in RViz by reading moveit.rviz file where Moveit enviroment data is configured.
You can control the mounted manipulator using an interactive marker, and simulate the motion of goal position, which helps preventing a possible physical contact by simulating the motion in advance.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```

![image](https://github.com/sanjiblama28/VIP/assets/92040822/fdf84ef4-e0ac-48a9-af91-68e7f823ef88)


## Run ROBOTIS GUI Controller

[Remote PC] You can also use ROBOTIS GUI to control the OpenMANIPULATOR-X in Gazebo. The GUI supports Task Space and Joint Space controls. Use any control methods you prefer.

Task Space Control: Control based on the valid gripping position (represented as a small red cube between the grippers) of the end-effector of the OpenMANIPULATOR-X.
Joint Space Control: Control based on each joint angle.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```

![image](https://github.com/sanjiblama28/VIP/assets/92040822/f4a3ee06-1696-4bcc-9ea9-c32fd6b46acd)
