cmake_minimum_required(VERSION 2.8.12.1)

PROJECT(project1A)
SET(VTK_DIR /Users/andrew/VTK-6.1.0)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(project1A MACOSX_BUNDLE project1A)

if(VTK_LIBRARIES)
  target_link_libraries(project1A ${VTK_LIBRARIES})
else()
  target_link_libraries(project1A vtkHybrid)
endif()

