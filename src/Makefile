
#####################################################################
# Copyright (c) 2008 Point Grey Research Inc.
#
# This Makefile is free software; Point Grey Research Inc.
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.
#
#####################################################################

ROOT_INCLUDE = /usr/local/include
ROOT_LIB = /usr/local/lib
LOCALLIB = ../include/pgrlibdcstereo

# compilation flags
CPPFLAGS += -I.

# libdc1394 installed in /usr/local/include location
CPPFLAGS += -I$(ROOT_INCLUDE)/dc1394
CPPFLAGS += -I$(LOCALLIB)
CPPFLAGS += -I$(ROOT_INCLUDE)/triclops
CPPFLAGS += -I/usr/local/include/opencv2
CPPFLAGS += -I/usr/local/Aria/include


CPPFLAGS += -I/usr/include/pcl-1.7	#pcl
CPPFLAGS += -I/usr/include/eigen3	#pcl	
CPPFLAGS += -I/usr/include/vtk-5.8	#pcl
#CPPFLAGS += `pkg-config --cflags mrpt-base mrpt-slam`
#CPPFLAGS += `pkg-config cvblob --cflags`
CPPFLAGS += -Wall -g
CPPFLAGS += -DLINUX
CPPFLAGS += -c -O1 -Wall -fPIC 
#CPPFLAGS += -Wall -O3

LDFLAGS	+= -L.
LDFLAGS	+= -L$(ROOT_LIB)/triclops
LDFLAGS += -L$(LOCALLIB)
LDFLAGS += -L$(ROOT_LIB)
LDFLAGS += -L/usr/local/Aria/lib 
LDFLAGS += -O1 -lAria -lpthread -ldl -lrt 
LIBS    += -ldc1394 -lraw1394 -pthread
LIBS	+= -lpgrlibdcstereo -ltriclops -lpnmutils -lflycapture -lflycapture2bridge -lm
LIBS	+= `pkg-config --libs opencv` #mrpt-base mrpt-slam`

LIBS	+= -lpcl_io -lboost_system -lpcl_common -lpcl_filters -lpcl_segmentation -lpcl_visualization -lvtkCommon -lvtkFiltering -lvtkRendering -lvtkGraphics	

LIBS	+= -lpcl_search -lpcl_kdtree -lpcl_octree -lpcl_features -lpcl_keypoints -lpcl_registration -lpcl_apps -lpcl_sample_consensus -lpcl_surface
#pcl . added all to avoid linking problem in the future.












CXX = g++-4.4


# Sources files
SRC = Robot.cpp \
	Camera.cpp \
	Map.cpp \
	SameObjectFinderColor.cpp \
	Surface.cpp View.cpp \
	ImageProcessing.cpp Color.cpp
	
SRC_MAIN = $(SRC) main.cpp

SRC_TEST_COLOR = $(SRC) test-color.cpp

# executable name and contents
EXEC		= guideAndMap
EXEC_TEST_COLOR = test-color





all: $(EXEC)

#to reset the camera
#	../../../libdc1394-2.0.2/examples/dc1394_reset_bus

$(EXEC): $(SRC_MAIN:%.cpp=../obj/%.o)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)
	
$(EXEC_TEST_COLOR): $(SRC_TEST_COLOR:%.cpp=../obj/%.o)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)


../obj/%.o:%.cpp
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $*.cpp -o ../obj/$*.o

clean:
	rm -f *~ ../obj/*.o *.d $(EXEC) *.pgm *.ppm
cleanimages:
	rm -f ../outputs/depth/*
	rm -f ../outputs/Views/*
	rm -f ../outputs/Maps/*
	rm -f ../outputs/rectified/*
	
cleanmaps:
	rm -f ../outputs/Maps/*



#####################################################################
#
# $Authors: Md Zulfikar Hossain and Martin Schrimpf (source: Guillaume Diallo-Mulliez)
# $Date: 28/08/2014
#
#####################################################################
