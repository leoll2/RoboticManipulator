cmake_minimum_required(VERSION 3.0.2)
project(pick_and_place_planner)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

# Install scripts
cs_install()

file(GLOB_RECURSE SCRIPTS scripts/**)
cs_install_scripts(${SCRIPTS})

cs_export()

