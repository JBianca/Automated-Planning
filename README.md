# Automated Planning: Interplanetary Museum Vault

## 📋 Project Overview
This project implements automated planning solutions for the **Interplanetary Museum Vault (IMV)** scenario, where autonomous robots must relocate fragile artifacts under various constraints. Five different planning approaches are implemented and evaluated.

## 🗂️ Project Structure
```
project/
├── problem1/          # Classical planning with single robot
├── problem2/          # Multi-robot planning with capacities
├── problem3/          # HTN planning with hierarchical decomposition
├── problem4/          # Temporal planning with durative actions
└── problem5/          # PlanSys2 integration with ROS2
```

## 🚀 Quick Start

### Prerequisites
- **Docker** installed and running
- **Java Runtime Environment** (for Problem 3)
- At least 4GB RAM available

### 1. Classical Planning (Problems 1, 2, 4)

```bash
# Start the planning environment
docker run -it --rm \
  --platform linux/amd64 \
  --privileged \
  -v /path/to/your/project:/project \
  myplanutils bash

# Navigate to the desired problem
cd /project/project/problem1  # or problem2/problem4

# Make test script executable and run
chmod +x test.sh
./test.sh
```

### 2. HTN Planning (Problem 3)

```bash
# Navigate to Problem 3 folder
cd /path/to/project/problem3

# Run PANDA planner
java -jar ../PANDA.jar -parser hddl ./domain.hddl ./problem.hddl
```

### 3. ROS2 PlanSys2 Integration (Problem 5)

```bash
# Start ROS2 environment
docker run -it --rm \
  -v /path/to/project/problem5:/root/plansys2_ws/src/problem5 \
  ros-humble bash

# Install tmux and prepare environment
apt-get update && apt-get install -y tmux
cd /root/plansys2_ws/src/problem5/
chmod +x tmux_session.sh
./tmux_session.sh
```

**In the tmux terminal:**
```bash
# Tab 1: Setup environment
source /opt/ros/humble/setup.bash
source ~/plansys2_ws/install/setup.bash
ros2 run plansys2_terminal plansys2_terminal

# Tab 2: Interact with PlanSys2
get plan      # Generate a plan
run           # Execute the plan
```

## 📁 File Descriptions

### Problem 1 (Classical Planning - Single Robot)
- `domain.pddl` - PDDL domain definition
- `problem1.pddl` - Problem instance
- `problem2.pddl` - Problem instance
- `test.sh` - Script to run various planners

### Problem 2 (Multi-Robot Planning)
- `domain.pddl` - Extended domain with robot capacities
- `problem1.pddl` - Multi-robot problem instance
- `problem2.pddl` - Multi-robot problem instance
- `test.sh` - Testing script

### Problem 3 (HTN Planning)
- `domain.hddl` - Hierarchical domain definition
- `problem.hddl` - HTN problem instance
- `test.sh` - Testing script
- `PANDA.jar` - HTN planner (place in parent directory)

### Problem 4 (Temporal Planning)
- `domain.pddl` - Temporal domain with durative actions
- `problem.pddl` - Temporal problem instance
- `test.sh` - Script for temporal planners

### Problem 5 (PlanSys2 Integration)
- `launch/problem5_launch.py` - ROS2 launch configuration
- `launch/commands` - Pre-configured terminal commands
- `pddl/domain.pddl` - PlanSys2-compatible domain
- `src/*.cpp` - ROS2 action executor nodes
- `CMakeLists.txt` - Build configuration for compiling action nodes
- `package.xml` - - ROS2 package dependencies and metadata
- `tmux_session.sh` - Automation script for execution

## 📊 Expected Outputs

### Problems 1-4
- **Planners used**: Fast Downward (FF, LAMA), POPF, OPTIC, TDF
- **Output**: Plan steps, execution time, expanded states
- **Metrics**: Plan length, makespan (temporal problems)

### Problem 5
- **PlanSys2 terminal output**: Plan generation and execution steps
- **Action progress**: 0-100% completion for each action
- **Final confirmation**: `[INFO] [executor]: Plan Succeeded`

## 📝 Notes for Evaluation
- All PDDL/HDDL files are validated and parsable
- Problem 5 demonstrates end-to-end planning to execution
- Each problem builds upon previous solutions
- Tested with multiple planners for comparison

## 🎯 Success Criteria
- ✅ All 5 problems generate valid plans
- ✅ Problem 5 executes successfully in PlanSys2
- ✅ Planners complete within reasonable time 
- ✅ Constraints are properly modeled and enforced

## 📄 License
Academic project for "Automated Planning: Theory and Practice" course, University of Trento, 2025-2026.

## 👥 Authors
Jie Chen (mat. 256177) - University of Trento