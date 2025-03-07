# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/ESP-IDF/Espressif/frameworks/esp-idf-v4.4.2/components/bootloader/subproject"
  "D:/Desktop/mcpwm_robot_walk3.7/build/bootloader"
  "D:/Desktop/mcpwm_robot_walk3.7/build/bootloader-prefix"
  "D:/Desktop/mcpwm_robot_walk3.7/build/bootloader-prefix/tmp"
  "D:/Desktop/mcpwm_robot_walk3.7/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Desktop/mcpwm_robot_walk3.7/build/bootloader-prefix/src"
  "D:/Desktop/mcpwm_robot_walk3.7/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Desktop/mcpwm_robot_walk3.7/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
