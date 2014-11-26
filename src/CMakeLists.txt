#it compiles, but camera doesn't working.

cmake_minimum_required(VERSION 2.8.4)
project(ImgPro)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "~/Hossain/albot2Mapping/src")

#find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(PCL 1.3 REQUIRED) #it includes all components
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


include_directories(/usr/local/Aria/include)
link_directories(/usr/local/Aria/lib)



include_directories(/usr/local/include)
link_directories(/usr/local/lib)


include_directories(../include/pgrlibdcstereo)
link_directories(../include/pgrlibdcstereo)



set(SOURCE_FILES Robot.cpp Camera.cpp Map.cpp SameSurfaceFinderColor.cpp 	SameSurfaceFinderOdo.cpp Surface.cpp View.cpp ImageProcessing.cpp Color.cpp Printer.cpp main.cpp)
add_executable(main ${SOURCE_FILES})


target_link_libraries(main ${PCL_LIBRARIES} ${OpenCV_LIBS} Aria.so dc1394.so pgrlibdcstereo.a triclops.so pnmutils.a) #it links all libraries
