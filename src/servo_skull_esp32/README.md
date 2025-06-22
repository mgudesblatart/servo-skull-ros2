# servo_skull

A mixed-language ROS 2 package for Raspberry Pi 5, containing both Python and C++ nodes.

## Building and Installing All Nodes (Python and C++)

1. **Ensure your Python packages are structured correctly:**
   - Each Python package directory (e.g., `servo_skull/servo_skull/`) must contain an `__init__.py` file (can be empty).
   - Example structure:
     ```
     src
     └── servo_skull_esp32/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── servo_skull_esp32/
           ├── __init__.py
           └── nodes/
              └── interface_node.py
     ```

2. **Install dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build all packages:**
   ```bash
   colcon build
   ```
   Or to build a specific package:
   ```bash
   colcon build --packages-select servo_skull_esp32
   ```

4. **Source your workspace:**
   ```bash
   source install/setup.bash
   ```

### Python Node
```bash
ros2 run servo_skull_esp32 interface_node
```

## Notes
- Make sure you have installed ROS 2 Humble (or later) for ARM64.
- Both nodes print a message every second to demonstrate functionality.
