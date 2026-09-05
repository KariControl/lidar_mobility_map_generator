cmake_minimum_required(VERSION 3.16)
project(lidar_mobility_map_generator VERSION 0.11.0 LANGUAGES C CXX)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" LMMG_PACKAGE_XML)
string(
  REGEX MATCH
  "<version>[ \t\r\n]*([^< \t\r\n]+)[ \t\r\n]*</version>"
  LMMG_PACKAGE_VERSION_MATCH
  "${LMMG_PACKAGE_XML}")
if(NOT LMMG_PACKAGE_VERSION_MATCH)
  message(FATAL_ERROR "Could not read the version from package.xml")
endif()
set(LMMG_PACKAGE_VERSION "${CMAKE_MATCH_1}")
if(NOT "${PROJECT_VERSION}" STREQUAL "${LMMG_PACKAGE_VERSION}")
  message(
    FATAL_ERROR
    "CMake project version ${PROJECT_VERSION} does not match package.xml "
    "version ${LMMG_PACKAGE_VERSION}")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Threads REQUIRED)

# This target exercises the real Autoware Local-coordinate loader and the
# Lanelet2 traffic/routing libraries.  It is optional so the map generator can
# still be built on a ROS-only workstation. Keep it opt-in, and fail at
# configure time when explicitly requested dependencies are unavailable.
option(
  LMMG_BUILD_AUTOWARE_LANELET_SMOKE
  "Build and install the real Autoware/Lanelet2 integration smoke checker"
  OFF)
if(LMMG_BUILD_AUTOWARE_LANELET_SMOKE)
  # autoware_map_loader exports pcl_conversions; Ubuntu's VTK/PCL config in
  # turn references MPI::MPI_C.  Define it before loading that dependency.
  find_package(MPI REQUIRED COMPONENTS C)
  find_package(autoware_map_loader REQUIRED)
  find_package(autoware_map_msgs REQUIRED)
  find_package(autoware_lanelet2_utils REQUIRED)
  find_package(lanelet2_core REQUIRED)
  find_package(lanelet2_routing REQUIRED)
  find_package(lanelet2_traffic_rules REQUIRED)
endif()

add_library(${PROJECT_NAME}_core
  src/types.cpp
  src/trajectory.cpp
  src/transform_graph.cpp
  src/pointcloud_io.cpp
  src/voxel_map.cpp
  src/occupancy_grid.cpp
  src/route_graph.cpp
  src/body_passage_planning.cpp
  src/observed_route_graph.cpp
  src/route_editor.cpp
  src/pipeline.cpp
  src/exporters.cpp
  src/navigation_outputs.cpp
  src/navigation_authoring.cpp
  src/navigation_promotion.cpp
  src/vector_map_source.cpp
  src/nav2_experimental.cpp
  src/nav2_route_export.cpp
  src/glim_reader.cpp
  src/review_io.cpp
  src/semantic_editor_web.cpp
  src/semantic_map.cpp
  src/semantic_route_graph.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(
  ${PROJECT_NAME}_core PRIVATE LMMG_PROJECT_VERSION="${PROJECT_VERSION}")
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_ros
  src/rosbag_reader.cpp
  src/ros_parameters.cpp
)
target_include_directories(${PROJECT_NAME}_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# ament_target_dependencies() uses CMake's plain target_link_libraries()
# signature on Jazzy.  Keep every call for this target in the same form.
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_core)
target_compile_features(${PROJECT_NAME}_ros PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_ros
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  tf2_msgs
)
target_compile_options(${PROJECT_NAME}_ros PRIVATE -Wall -Wextra -Wpedantic)

add_executable(generate_vector_map src/main.cpp)
target_link_libraries(generate_vector_map
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_ros
)
ament_target_dependencies(generate_vector_map rclcpp)
target_compile_options(generate_vector_map PRIVATE -Wall -Wextra -Wpedantic)

# Product-named generator executable.  The descriptive generate_vector_map
# command is also installed for the focused Vector Map workflow.
add_executable(lidar_mobility_map_generator src/main.cpp)
target_link_libraries(lidar_mobility_map_generator
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_ros
)
ament_target_dependencies(lidar_mobility_map_generator rclcpp)
target_compile_options(
  lidar_mobility_map_generator PRIVATE -Wall -Wextra -Wpedantic)

add_executable(semantic_map_editor src/semantic_editor_server.cpp)
target_link_libraries(semantic_map_editor
  PRIVATE
    ${PROJECT_NAME}_core
    Threads::Threads
)
target_compile_options(semantic_map_editor PRIVATE -Wall -Wextra -Wpedantic)

add_executable(review_vector_map src/review_node.cpp)
target_link_libraries(review_vector_map ${PROJECT_NAME}_core)
ament_target_dependencies(review_vector_map
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)
target_compile_options(review_vector_map PRIVATE -Wall -Wextra -Wpedantic)

# Product-named RViz review publisher.  The descriptive review_vector_map
# command is also installed for the focused Vector Map workflow.
add_executable(lidar_mobility_map_review src/review_node.cpp)
target_link_libraries(lidar_mobility_map_review ${PROJECT_NAME}_core)
ament_target_dependencies(lidar_mobility_map_review
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)
target_compile_options(
  lidar_mobility_map_review PRIVATE -Wall -Wextra -Wpedantic)

if(LMMG_BUILD_AUTOWARE_LANELET_SMOKE)
  add_executable(autoware_lanelet_smoke tools/autoware_lanelet_smoke.cpp)
  target_include_directories(
    autoware_lanelet_smoke BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(autoware_lanelet_smoke ${PROJECT_NAME}_core)
  ament_target_dependencies(autoware_lanelet_smoke
    autoware_lanelet2_utils
    autoware_map_loader
    autoware_map_msgs
    lanelet2_core
    lanelet2_routing
    lanelet2_traffic_rules
    rclcpp
  )
  target_compile_options(autoware_lanelet_smoke PRIVATE -Wall -Wextra -Wpedantic)
install(
    TARGETS autoware_lanelet_smoke
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  set(LMMG_AUTOWARE_LANELET_SMOKE_AVAILABLE TRUE)
else()
  set(LMMG_AUTOWARE_LANELET_SMOKE_AVAILABLE FALSE)
endif()

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_ros
    generate_vector_map
    lidar_mobility_map_generator
    review_vector_map
    lidar_mobility_map_review
    semantic_map_editor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  FILES
    config/glim.yaml
    config/rosbag2.yaml
    config/review.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(
  FILES
    launch/edit_and_review.launch.py
    launch/edit_navigation_map.launch.py
    launch/edit_vector_map.launch.py
    launch/generate.launch.py
    launch/generate_and_review.launch.py
    launch/review.launch.py
    launch/semantic_editor.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
install(
  FILES
    rviz/review.rviz
  DESTINATION share/${PROJECT_NAME}/rviz)
install(
  FILES
    docs/operator_manual.md
    docs/operator_manual_ja.md
  DESTINATION share/${PROJECT_NAME}/docs)
install(
  FILES
    docs/images/autoware_lanelet2_rviz2.png
    docs/images/autoware_vector_map_driving_rviz2.png
    docs/images/logo.png
    docs/images/mid360_input_pointcloud_ja.png
    docs/images/navigation_map_editor_en.png
    docs/images/navigation_map_editor_ja.png
    docs/images/navigation_map_rviz2.png
    docs/images/vector_map_editor_en.png
    docs/images/vector_map_editor_ja.png
    docs/images/vector_map_input_pointcloud_overview.png
    docs/images/vector_map_manual_route_ja.png
    docs/images/vector_map_output_ja.png
    docs/images/vector_map_stop_line_ja.png
    docs/images/vector_map_target_route_ja.png
  DESTINATION share/${PROJECT_NAME}/docs/images)
install(
  FILES
    docs/videos/autoware_vector_map_driving_rviz2.webm
  DESTINATION share/${PROJECT_NAME}/docs/videos)
install(
  FILES README.md README_ja.md LICENSE CHANGELOG.rst TRADEMARKS.md
  DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/check_vector_map_edit_smoke.py
    scripts/check_roscon_demo.py
    scripts/create_roscon_demo_input.py
    scripts/follow_nav2_waypoints.py
    scripts/generation_calibration_contract.py
    scripts/resolve_autoware_vehicle_info.py
    scripts/verify_target_vehicle_map_binding.py
    scripts/run_roscon_demo.sh
    scripts/run_vector_map_workflow.sh
    scripts/stage_autoware_closed_course_map.sh
    scripts/validate_autoware_candidate.py
  DESTINATION lib/${PROJECT_NAME})

# Development workspaces may include the private acceptance-result viewer.
# Keep the public source archive independent: the release tree intentionally
# omits these files, so install them only when the complete local viewer is
# present.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch/autoware_result_review.launch.py" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz/autoware_result_review.rviz" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/review_autoware_planning_control_result.sh" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/replay_autoware_planning_control_result.py" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/autoware_result_replay_model.py" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/autoware_result_vehicle_mesh.py")
  install(
    FILES launch/autoware_result_review.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
  install(
    FILES rviz/autoware_result_review.rviz
    DESTINATION share/${PROJECT_NAME}/rviz)
  install(
    PROGRAMS
      scripts/review_autoware_planning_control_result.sh
      scripts/replay_autoware_planning_control_result.py
      scripts/autoware_result_replay_model.py
      scripts/autoware_result_vehicle_mesh.py
    DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core PRIVATE ${PROJECT_NAME}_core)
  target_compile_definitions(
    test_core PRIVATE LMMG_PROJECT_VERSION="${PROJECT_VERSION}")
  target_compile_options(test_core PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME lidar_mobility_map_generator_core COMMAND test_core)
  add_executable(test_observed_route_graph test/test_observed_route_graph.cpp)
  target_link_libraries(test_observed_route_graph PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(test_observed_route_graph PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME lidar_mobility_map_generator_observed_route_graph
    COMMAND test_observed_route_graph)
  add_executable(test_evidence_model test/test_evidence_model.cpp)
  target_link_libraries(test_evidence_model PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(test_evidence_model PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME lidar_mobility_map_generator_evidence_model COMMAND test_evidence_model)
  add_executable(test_free_space test/test_free_space.cpp)
  target_link_libraries(test_free_space PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(test_free_space PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME lidar_mobility_map_generator_free_space COMMAND test_free_space)
  add_executable(test_body_passage_planning test/test_body_passage_planning.cpp)
  target_link_libraries(test_body_passage_planning PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(test_body_passage_planning PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME lidar_mobility_map_generator_body_passage_planning
    COMMAND test_body_passage_planning)
  add_executable(
    test_lanelet_boundary_serialization
    test/test_lanelet_boundary_serialization.cpp)
  target_link_libraries(test_lanelet_boundary_serialization PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(
    test_lanelet_boundary_serialization PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME lidar_mobility_map_generator_lanelet_boundary_serialization
    COMMAND test_lanelet_boundary_serialization)
  add_executable(test_nav2_experimental test/test_nav2_experimental.cpp)
  target_link_libraries(test_nav2_experimental PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(test_nav2_experimental PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME lidar_mobility_map_generator_nav2_experimental
    COMMAND test_nav2_experimental)
  add_executable(test_semantic_route_graph test/test_semantic_route_graph.cpp)
  target_link_libraries(test_semantic_route_graph PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(test_semantic_route_graph PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME lidar_mobility_map_generator_semantic_route_graph
    COMMAND test_semantic_route_graph)
  add_executable(test_navigation_authoring test/test_navigation_authoring.cpp)
  target_link_libraries(test_navigation_authoring PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(test_navigation_authoring PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME lidar_mobility_map_generator_navigation_authoring
    COMMAND test_navigation_authoring)
  add_executable(test_vector_map_source test/test_vector_map_source.cpp)
  target_link_libraries(test_vector_map_source PRIVATE ${PROJECT_NAME}_core)
  target_compile_options(test_vector_map_source PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME lidar_mobility_map_generator_vector_map_source
    COMMAND test_vector_map_source)
  add_executable(test_ros_parameters test/test_ros_parameters.cpp)
  target_link_libraries(test_ros_parameters ${PROJECT_NAME}_ros)
  ament_target_dependencies(test_ros_parameters rclcpp)
  target_compile_options(test_ros_parameters PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME lidar_mobility_map_generator_ros_parameters COMMAND test_ros_parameters)
  set_tests_properties(
    lidar_mobility_map_generator_ros_parameters
    PROPERTIES ENVIRONMENT "ROS_LOG_DIR=/tmp")
  add_test(
    NAME lidar_mobility_map_generator_review_launch
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_review_launch.py
  )
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_result_replay.py")
    add_test(
      NAME lidar_mobility_map_generator_autoware_result_replay
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_result_replay.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/autoware_result_replay_model.py
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_result_vehicle_mesh.py" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/autoware_result_vehicle_mesh.py")
    add_test(
      NAME lidar_mobility_map_generator_autoware_result_vehicle_mesh
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_result_vehicle_mesh.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/autoware_result_vehicle_mesh.py
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docker/acceptance/test_static.py")
    add_test(
      NAME lidar_mobility_map_generator_autoware_full_route_scenario
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/docker/acceptance/test_static.py
        AutowareMissionDerivationTest
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_speed_limit_runtime_audit.py")
    add_test(
      NAME lidar_mobility_map_generator_autoware_speed_limit_runtime_audit
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_speed_limit_runtime_audit.py
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_stock_runner_static.py")
    add_test(
      NAME lidar_mobility_map_generator_autoware_stock_runner_static
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_stock_runner_static.py
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_strict_policy.py" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_autoware_strict_policy.py" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_autoware_full_route_planning_control_test.sh" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_autoware_stock_planning_control_test.sh")
    add_test(
      NAME lidar_mobility_map_generator_autoware_strict_policy
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_strict_policy.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_autoware_strict_policy.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_autoware_full_route_planning_control_test.sh
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_autoware_stock_planning_control_test.sh
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_full_route_planning_control_matrix.py")
    add_test(
      NAME lidar_mobility_map_generator_autoware_full_route_matrix
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_full_route_planning_control_matrix.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_autoware_full_route_planning_control_matrix.sh
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/audit_autoware_full_route_planning_control_matrix.py
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_runtime_provenance.py")
    add_test(
      NAME lidar_mobility_map_generator_autoware_runtime_provenance
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_runtime_provenance.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/autoware_runtime_provenance.py
    )
  endif()
  add_test(
    NAME lidar_mobility_map_generator_follow_nav2_waypoints
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_follow_nav2_waypoints.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/follow_nav2_waypoints.py
  )
  add_test(
    NAME lidar_mobility_map_generator_nav2_load_only_acceptance_static
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_nav2_load_only_acceptance.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_nav2_load_only_acceptance.sh
      ${CMAKE_CURRENT_SOURCE_DIR}/docker/acceptance/scripts/run_nav2_load_only_acceptance.sh
      ${CMAKE_CURRENT_SOURCE_DIR}/docker/acceptance/scripts/nav2_load_only_probe.py
  )
  add_test(
    NAME lidar_mobility_map_generator_generation_fail_closed
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generation_fail_closed.py
      $<TARGET_FILE:generate_vector_map>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/glim.yaml
  )
  add_test(
    NAME lidar_mobility_map_generator_autoware_stage
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_stage.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/stage_autoware_closed_course_map.sh
  )
  add_test(
    NAME lidar_mobility_map_generator_validate_autoware_candidate
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_validate_autoware_candidate.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_autoware_candidate.py
  )
  add_test(
    NAME lidar_mobility_map_generator_vector_map_manual_editor
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vector_map_manual_editor.py
      $<TARGET_FILE:semantic_map_editor>
      ${CMAKE_CURRENT_SOURCE_DIR}/src/semantic_editor_web.cpp
  )
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_planning_test_gate.py" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_autoware_planning_test_gate.py")
    add_test(
      NAME lidar_mobility_map_generator_autoware_planning_test_gate
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_autoware_planning_test_gate.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_autoware_planning_test_gate.py
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_semantic_editor_readiness.py")
    add_test(
      NAME lidar_mobility_map_generator_semantic_editor_readiness
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_semantic_editor_readiness.py
        $<TARGET_FILE:semantic_map_editor>
        ${CMAKE_CURRENT_SOURCE_DIR}/src/semantic_editor_web.cpp
    )
  endif()
  add_test(
    NAME lidar_mobility_map_generator_roscon_demo_fixture
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_roscon_demo_fixture.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/create_roscon_demo_input.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_roscon_demo.py
  )
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_maximal_autoware_authoring_fixture.py")
    add_test(
      NAME lidar_mobility_map_generator_maximal_autoware_authoring_fixture
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_maximal_autoware_authoring_fixture.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/create_maximal_autoware_authoring_fixture.py
    )
  endif()
  add_test(
    NAME lidar_mobility_map_generator_generation_calibration_contract_direct
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generation_calibration_contract_direct.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generation_calibration_contract.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_vector_map_workflow.sh
  )
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_file_reference.py" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs/cpp_file_reference.md" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs/cpp_file_reference_ja.md")
    add_test(
      NAME lidar_mobility_map_generator_cpp_file_reference
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_file_reference.py
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/docs/cpp_file_reference.md
        ${CMAKE_CURRENT_SOURCE_DIR}/docs/cpp_file_reference_ja.md
    )
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_public_release_tools.py" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/audit_public_release.py" AND
      EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/create_public_release_tree.py")
    add_test(
      NAME lidar_mobility_map_generator_public_release_tools
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_public_release_tools.py
    )
  endif()
  add_test(
    NAME lidar_mobility_map_generator_vector_map_edit_smoke
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vector_map_edit_smoke.py
  )
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_release_surface.py")
    add_test(
      NAME lidar_mobility_map_generator_release_surface
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_release_surface.py
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()
  # Development-only data-campaign tests live outside the public manifest.
  # The optional include preserves them in this working tree without making a
  # clean public checkout depend on private site locks, datasets, or profiles.
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/local_validation_tests.cmake" OPTIONAL)
  if(LMMG_AUTOWARE_LANELET_SMOKE_AVAILABLE)
    add_test(
      NAME lidar_mobility_map_generator_autoware_lanelet_smoke
      COMMAND $<TARGET_FILE:autoware_lanelet_smoke>
        ${CMAKE_CURRENT_SOURCE_DIR}/test/data/autoware_local_chain.osm
    )
    set_tests_properties(
      lidar_mobility_map_generator_autoware_lanelet_smoke
      PROPERTIES ENVIRONMENT "ROS_LOG_DIR=/tmp")
    add_test(
      NAME lidar_mobility_map_generator_autoware_authored_stop_smoke
      COMMAND $<TARGET_FILE:autoware_lanelet_smoke> --self-test-authored-stop
    )
    set_tests_properties(
      lidar_mobility_map_generator_autoware_authored_stop_smoke
      PROPERTIES ENVIRONMENT "ROS_LOG_DIR=/tmp")
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  std_msgs
  std_srvs
  tf2_msgs
  visualization_msgs
)
ament_package()
