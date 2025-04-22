cmake_minimum_required(VERSION 3.8)
project(attachable_joint_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(gz-physics7 REQUIRED)
set(IGN_MSGS_VER ${gz-physics7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED)
set(IGN_MSGS_VER ${gz-common5_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
add_library(attachable_joint_plugin SHARED src/attachable_joint_plugin.cc include/attachable_joint_plugin.hh)
set_property(TARGET attachable_joint_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(attachable_joint_plugin
  PRIVATE gz-plugin${IGN_PLUGIN_VER}::gz-plugin${IGN_PLUGIN_VER}
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gz-common5::gz-common5
  PRIVATE gz-physics7::gz-physics7
  )
target_include_directories(attachable_joint_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
