<center>
    <h1>Tiago Robot Setup</h1>
</center>

---

</br>

## Setup Steps

**1. Create a workspace:** 
    ```
    source /opt/ros/humble/setup.bash
    mkdir -p ~/my_ros2_ws/src
    cd ~/my_ros2_ws/src
    ```

**2. Clone the GitHub repository of PAL Robotic's Tiago Robot:** 
    ```
    git clone https://github.com/pal-robotics/tiago_robot
    ```

**3. Resolve dependencies** 
    ```
    cd ~/my_ros2_ws
    rosdep install -i --from-path src --rosdistro humble -y
    ```

**4. Build the workspace** 
    Ensure that you are in the root of the workspace and not in `/src`
    
    ```
    colcon build --symlink-install
    ```

**5. Source the workspace** 
    ```
    source install/setup.bash 
    ```

**6. Launch Tiago Robot in RViz** 
    ```
    ros2 launch tiago_description show.launch.py 
    ```

**7. Echo the `/robot_description` topic** 
    ```
    ros2 topic echo /robot_description
    ```

<p></br><b><i>Hooray! This completes the tutorial.</i></b></p>

</br>

### Video Demonstration
Here is the YouTube link- *[Video](https://youtu.be/jhSbuHDeLcM?si=LFhFCSqqGm0c4x4k)*
---

</br>

---