## 📦 Build and Setup Instructions

1. **Put this into any ROS 2 workspace**  
   (Example: `~/ros2_ws/src`)

   ```bash
   cd ~/ros2_ws/src
   # Copy your package folder here
   ```

2. **Build the workspace**

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. **Source the workspace**

   ```bash
   cd week4_arm
   source install/setup.bash
   ```

4. **Make launch scripts executable**

   ```bash
   cd launch
   chmod +x *.py
   ```


## 🖥️ RViz2 Setup Instructions

1. **Launch the robot model in RViz2**

   ```bash
   ros2 launch week4_arm display_arm.launch.py
   ```

2. **In RViz2:**
   - Click on **Global Options** in the left panel.
   - Set **Fixed Frame** to `base_link` .

3. **Add the Robot Model:**
   - Click **Add** at the bottom left panel.
   - Choose **RobotModel**.
   - In the **Description Topic**, set it to `robot_description`.

   ✅ You should now see the 2D robot arm model in the display window.

4. *(Optional)* **Add TF to visualize frames:**
   - Click **Add**.
   - Choose **TF** to visualize the coordinate frames of your robot.
   - Red is the XR-axis, green is the YR-axis, blue is the ZR-axis. 
