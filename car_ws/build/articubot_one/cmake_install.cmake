<<<<<<< HEAD
# Install script for directory: /home/houndsito/vt-cro/robogamescode/car_ws/src/tmp
=======
# Install script for directory: /home/lab/vt-cro/robogamescode/car_ws/src/tmp
>>>>>>> bffa34cc4d2e80b9ee4c00295ab6b34fe4980d59

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/houndsito/vt-cro/robogamescode/car_ws/install/articubot_one")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/articubot_one" TYPE PROGRAM FILES "/home/houndsito/vt-cro/robogamescode/car_ws/src/tmp/src/depth_to_scan.py")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/articubot_one" TYPE PROGRAM FILES "/home/lab/vt-cro/robogamescode/car_ws/src/tmp/src/depth_to_scan.py")
>>>>>>> bffa34cc4d2e80b9ee4c00295ab6b34fe4980d59
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one" TYPE DIRECTORY FILES
<<<<<<< HEAD
    "/home/houndsito/vt-cro/robogamescode/car_ws/src/tmp/config"
    "/home/houndsito/vt-cro/robogamescode/car_ws/src/tmp/description"
    "/home/houndsito/vt-cro/robogamescode/car_ws/src/tmp/launch"
    "/home/houndsito/vt-cro/robogamescode/car_ws/src/tmp/worlds"
    "/home/houndsito/vt-cro/robogamescode/car_ws/src/tmp/maps"
=======
    "/home/lab/vt-cro/robogamescode/car_ws/src/tmp/config"
    "/home/lab/vt-cro/robogamescode/car_ws/src/tmp/description"
    "/home/lab/vt-cro/robogamescode/car_ws/src/tmp/launch"
    "/home/lab/vt-cro/robogamescode/car_ws/src/tmp/worlds"
    "/home/lab/vt-cro/robogamescode/car_ws/src/tmp/maps"
>>>>>>> bffa34cc4d2e80b9ee4c00295ab6b34fe4980d59
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/articubot_one")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/articubot_one")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one/environment" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one/environment" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_index/share/ament_index/resource_index/packages/articubot_one")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one/cmake" TYPE FILE FILES
    "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_core/articubot_oneConfig.cmake"
    "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/ament_cmake_core/articubot_oneConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one" TYPE FILE FILES "/home/houndsito/vt-cro/robogamescode/car_ws/src/tmp/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/articubot_one" TYPE FILE FILES "/home/lab/vt-cro/robogamescode/car_ws/src/tmp/package.xml")
>>>>>>> bffa34cc4d2e80b9ee4c00295ab6b34fe4980d59
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/houndsito/vt-cro/robogamescode/car_ws/build/articubot_one/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
