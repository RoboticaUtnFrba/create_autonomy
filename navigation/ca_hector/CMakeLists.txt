cmake_minimum_required(VERSION 2.8.3)
project(ca_hector)

find_package(catkin REQUIRED COMPONENTS
  hector_geotiff
  hector_mapping
)

catkin_package()
