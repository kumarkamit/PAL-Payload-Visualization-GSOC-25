<center>
    <h1>Using Pinocchio with Tiago Robot</h1>
</center>

## Steps to create a Pinocchio model, perform forward kinematics, get Jacobian at a frame, and check collision between links 

**1. Install Pinocchio for ROS Humble** 
    
    sudo apt install ros-$ROS_DISTRO-pinocchio



**2. Clone my ROS package `tiago_pinoc` into the `src` of your workspace:**
Take a look at the `package.xml` and `CMakeLists.txt` file. A python file, `scripts/my_pinoccohio_node.py` is created to perform the task.

**3. Run the Python File** 
This python file will wait for the `/robot_description` topic and the `/joint_states` to be available.
    
    ros2 run tiago_pinoc my_pinoccohio_node.py
    

**4. Launch Tiago Robot in RViz** 
    
    ros2 launch tiago_description show.launch.py
    
You can observe the changes in the transformation matrix of `arm_7_joint` frame, Jacobian and the collision between all the links as the joint angles are changed through the toolbar.


<p></br><b><i>Hooray! This completes the tutorial.</i></b></p>

</br>

### Video Demonstration
Here is the YouTube link- *[Video](https://youtu.be/kzp-68Wxv74)*
---

</br>

---
