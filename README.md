Name  Mohammadhossein baba S5919466

Welcome to the second assignment for the Research Track 1 course. This assignment focuses on creating a new package that contains three nodes responsible for controlling robot movement in a specific environment and gathering relevant data. Let's dive into the details:

### Nodes:

1. **node_a.py**: This node implements an action client, allowing the user to set a target position (x, y) for the robot or cancel the current target. It utilizes the feedback and status of the action server to determine when the target has been reached. Additionally, this node publishes the robot's position and velocity as a custom message (x, y, vel_x, vel_z) based on the values published on the `/odom` topic.

2. **node_b.py**: This node provides a service that returns the coordinates of the last target position sent by the user. When called, the service named `/input` retrieves and returns the target positions.

To visualize the last desired position of the robot, you can call the service using the following command in a new terminal:
```bash
$ rosservice call /input
```

3. **node_c.py**: This node subscribes to the robot's position and velocity using the custom message published on the `/pos_vel` topic. It implements a server that retrieves the distance between the robot and the target position, as well as the robot's average speed. The service named `/info_service` provides these values upon request.

### Installation and Execution:

To run the simulator, follow these steps:

1. Ensure you have ROS installed on your system.

2. Clone the repository [RT1_assignment_2](https://github.com/LemmaMoto/RT1_assignment_2.git) using the command:
```bash
$ git clone https://github.com/LemmaMoto/RT1_assignment_2.git
```

3. Install `xterm` by executing the following command:
```bash
$ sudo apt-get -y install xterm
```

4. Grant execution permissions to the Python files inside the `scripts` folder using the following commands:
```bash
$ chmod +x node_a.py
$ chmod +x node_b.py
$ chmod +x node_c.py
```

5. To launch the program and initiate the simulation, execute the following command:
```bash
$ roslaunch assignment_2_2023 assignment1.launch
```
