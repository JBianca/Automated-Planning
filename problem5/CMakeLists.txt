cmake_minimum_required(VERSION 3.5)
project(problem5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(seal_action_node src/seal_action_node.cpp)
ament_target_dependencies(seal_action_node ${dependencies})

add_executable(unseal_action_node src/unseal_action_node.cpp)
ament_target_dependencies(unseal_action_node ${dependencies})

add_executable(pick_hall_alpha_action_node src/pick_hall_alpha_action_node.cpp)
ament_target_dependencies(pick_hall_alpha_action_node ${dependencies})

add_executable(pick_hall_beta_action_node src/pick_hall_beta_action_node.cpp)
ament_target_dependencies(pick_hall_beta_action_node ${dependencies})

add_executable(drop_at_cryo_action_node src/drop_at_cryo_action_node.cpp)
ament_target_dependencies(drop_at_cryo_action_node ${dependencies})

add_executable(drop_at_pod_action_node src/drop_at_pod_action_node.cpp)
ament_target_dependencies(drop_at_pod_action_node ${dependencies})

add_executable(cool_artifact_action_node src/cool_artifact_action_node.cpp)
ament_target_dependencies(cool_artifact_action_node ${dependencies})

add_executable(activate_antivibration_action_node src/activate_antivibration_action_node.cpp)
ament_target_dependencies(activate_antivibration_action_node ${dependencies})

add_executable(pick_from_cryo_action_node src/pick_from_cryo_action_node.cpp)
ament_target_dependencies(pick_from_cryo_action_node ${dependencies})

add_executable(pick_from_pod_action_node src/pick_from_pod_action_node.cpp)
ament_target_dependencies(pick_from_pod_action_node ${dependencies})

add_executable(deliver_to_stasis_action_node src/deliver_to_stasis_action_node.cpp)
ament_target_dependencies(deliver_to_stasis_action_node ${dependencies})

add_executable(stabilize_action_node src/stabilize_action_node.cpp)
ament_target_dependencies(stabilize_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  seal_action_node
  unseal_action_node
  pick_hall_alpha_action_node
  pick_hall_beta_action_node
  drop_at_cryo_action_node
  drop_at_pod_action_node
  cool_artifact_action_node
  activate_antivibration_action_node
  pick_from_cryo_action_node
  pick_from_pod_action_node
  deliver_to_stasis_action_node
  stabilize_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})
ament_package()