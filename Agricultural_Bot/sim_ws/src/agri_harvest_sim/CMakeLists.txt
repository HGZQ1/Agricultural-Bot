cmake_minimum_required(VERSION 3.16)
project(agri_harvest_sim)
find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(nlohmann_json REQUIRED)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz_plugin_path.dsv.in")
add_library(agri_harvest_system SHARED src/HarvestSystem.cc)
target_compile_features(agri_harvest_system PRIVATE cxx_std_17)
target_link_libraries(agri_harvest_system PRIVATE gz-sim8::gz-sim8
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
add_library(agri_motion_diagnostics SHARED src/MotionDiagnostics.cc)
target_compile_features(agri_motion_diagnostics PRIVATE cxx_std_17)
target_link_libraries(agri_motion_diagnostics PRIVATE gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
add_library(agri_arm_joint_diagnostics_v1 SHARED src/ArmJointDiagnostics.cc)
target_compile_features(agri_arm_joint_diagnostics_v1 PRIVATE cxx_std_17)
target_link_libraries(agri_arm_joint_diagnostics_v1 PRIVATE gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
add_library(agri_ideal_steering_system SHARED src/IdealSteeringSystem.cc)
target_compile_features(agri_ideal_steering_system PRIVATE cxx_std_17)
target_link_libraries(agri_ideal_steering_system PRIVATE gz-sim8::gz-sim8
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
add_library(agri_ideal_four_wheel_steering_system SHARED src/IdealFourWheelSteeringSystem.cc)
target_compile_features(agri_ideal_four_wheel_steering_system PRIVATE cxx_std_17)
target_link_libraries(agri_ideal_four_wheel_steering_system PRIVATE gz-sim8::gz-sim8
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
add_library(agri_fruit_arm_pass_through SHARED src/FruitArmPassThroughSystem.cc)
target_compile_features(agri_fruit_arm_pass_through PRIVATE cxx_std_17)
target_link_libraries(agri_fruit_arm_pass_through PRIVATE gz-sim8::gz-sim8
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
add_library(agri_gripper_plant_pass_through SHARED src/GripperPlantPassThroughSystem.cc)
target_compile_features(agri_gripper_plant_pass_through PRIVATE cxx_std_17)
target_link_libraries(agri_gripper_plant_pass_through PRIVATE gz-sim8::gz-sim8
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
add_library(agri_native_joint_limit_roundoff_recovery SHARED src/NativeJointLimitRoundoffRecovery.cc)
target_compile_features(agri_native_joint_limit_roundoff_recovery PRIVATE cxx_std_17)
target_link_libraries(agri_native_joint_limit_roundoff_recovery PRIVATE gz-sim8::gz-sim8
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
add_library(agri_ideal_gripper_system SHARED src/IdealGripperSystem.cc)
target_compile_features(agri_ideal_gripper_system PRIVATE cxx_std_17)
target_link_libraries(agri_ideal_gripper_system PRIVATE gz-sim8::gz-sim8
  gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
install(TARGETS agri_harvest_system agri_motion_diagnostics agri_ideal_steering_system
  agri_ideal_four_wheel_steering_system
  agri_fruit_arm_pass_through agri_gripper_plant_pass_through agri_arm_joint_diagnostics_v1
  agri_native_joint_limit_roundoff_recovery agri_ideal_gripper_system LIBRARY DESTINATION lib)
install(PROGRAMS scripts/harvest_scene.py scripts/make_harvestable_world.py scripts/ideal_r2_steering.py
  scripts/fruit_arm_pass_through.py scripts/gripper_plant_pass_through.py scripts/ideal_gripper.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/scene_geometry.py scripts/gazebo_transport.py scripts/ideal_steering_command.py
  scripts/ideal_status_process.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(test_grip_evidence test/test_grip_evidence.cc)
  target_compile_features(test_grip_evidence PRIVATE cxx_std_17)
  add_test(NAME grip_evidence COMMAND test_grip_evidence)
  add_executable(test_basket_evidence test/test_basket_evidence.cc)
  target_compile_features(test_basket_evidence PRIVATE cxx_std_17)
  add_test(NAME basket_evidence COMMAND test_basket_evidence)
  add_executable(test_ideal_steering test/test_ideal_steering.cc)
  target_compile_features(test_ideal_steering PRIVATE cxx_std_17)
  target_link_libraries(test_ideal_steering PRIVATE gz-sim8::gz-sim8
    gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
  add_test(NAME ideal_steering COMMAND test_ideal_steering)
  add_executable(test_gripper_plant_pass_through test/test_gripper_plant_pass_through.cc)
  target_compile_features(test_gripper_plant_pass_through PRIVATE cxx_std_17)
  target_compile_options(test_gripper_plant_pass_through PRIVATE -UNDEBUG)
  target_link_libraries(test_gripper_plant_pass_through PRIVATE gz-sim8::gz-sim8
    gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-plugin2::register nlohmann_json::nlohmann_json)
  add_test(NAME gripper_plant_pass_through COMMAND test_gripper_plant_pass_through)
  ament_add_pytest_test(scene_geometry test/test_scene_geometry.py)
  ament_add_pytest_test(basket_floor test/test_basket_floor.py)
  ament_add_pytest_test(world_conversion test/test_world_conversion.py)
  ament_add_pytest_test(contact_transaction test/test_contact_transaction.py)
  ament_add_pytest_test(gazebo_transport test/test_gazebo_transport.py)
  ament_add_pytest_test(ideal_steering_command test/test_ideal_steering_command.py)
  ament_add_pytest_test(ideal_r2_steering_bridge test/test_ideal_r2_steering_bridge.py)
  ament_add_pytest_test(ideal_status_process test/test_ideal_status_process.py)
endif()
ament_package()
