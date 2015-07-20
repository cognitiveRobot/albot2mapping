
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
CPPFLAGS += -I$(ROOT_INCLUDE)/dc1394	#camera
CPPFLAGS += -I$(ROOT_INCLUDE)/triclops	#camera

CPPFLAGS += -I$(LOCALLIB)

CPPFLAGS += -I/usr/local/Aria/include	#Aria Robot

CPPFLAGS += -I/usr/local/include/opencv2    # openCV

CPPFLAGS += -I/usr/include/pcl-1.7	#pcl
CPPFLAGS += -I/usr/include/eigen3	#pcl	
CPPFLAGS += -I/usr/include/vtk-5.8	#pcl
#CPPFLAGS += `pkg-config --cflags mrpt-base mrpt-slam`
#CPPFLAGS += `pkg-config cvblob --cflags`
CPPFLAGS += -Wall -g -fno-inline
CPPFLAGS += -DLINUX
CPPFLAGS += -c -O1 -Wall -fPIC
#CPPFLAGS += -Wall -O3

LDFLAGS	+= -L.
LDFLAGS	+= -L$(ROOT_LIB)
LDFLAGS	+= -L$(ROOT_LIB)/triclops

LDFLAGS	+= -L$(LOCALLIB)

LDFLAGS	+= -L/usr/local/Aria/lib	    #Aria Robot
LDFLAGS    += -O1 -lAria -lpthread -ldl -lrt -lboost_system -lGL

LIBS	+= -ldc1394 -lraw1394 -pthread
LIBS	+= -lpgrlibdcstereo -ltriclops -lpnmutils -lflycapture -lflycapture2bridge -lm
LIBS	+= `pkg-config --libs opencv` #mrpt-base mrpt-slam`

LIBS	+= -lpcl_io -lpcl_common -lpcl_filters -lpcl_segmentation -lpcl_visualization -lvtkCommon -lvtkFiltering -lvtkRendering -lvtkGraphics
LIBS	+= -lpcl_search -lpcl_kdtree -lpcl_octree -lpcl_features -lpcl_keypoints -lpcl_registration -lpcl_apps -lpcl_sample_consensus -lpcl_surface
#pcl . added all to avoid linking problem in the future.



CXX = g++-4.4


# Sources files
LIBSRCS = \
	Robot.cpp \
	PointAndSurface.cpp \
	GeometryFuncs.cpp \
	Laser2Surface.cpp \
	Camera.cpp \
	Surface.cpp \
	View.cpp \
	Color.cpp \
	Map.cpp \
	SameSurfaceFinderColor.cpp \
	SameSurfaceFinderOdo.cpp \
	Printer.cpp \
	PathFinder.cpp \
	ImageProcessing.cpp \
	GlobalMap.cpp \
	Node.cpp

#main programs	
PROGSRCS= \
	offlineMappingUpdateAllSteps.cpp \
	GuidedXnMapping.cpp \
	offlineGuidedXnMapping.cpp \
	onlineMapping.cpp \
	minhMapping.cpp \
	findWayHome.cpp
#	offlineMappingFromLS.cpp \
#	testStereoMatching.cpp \
#	renameFiles.cpp \
#	robotControl.cpp
#	main.cpp \
#	AutoXplorationNMapping.cpp \
#	gotoActionExample.cpp
#	test-color.cpp \
#	test-surfacefinder.cpp \


LIBOBJS = $(LIBSRCS:%.cpp=../obj/%.o)
PROGOBJS = $(PROGSRCS:%.cpp=../obj/%.o)
LIBDEPS = $(PROGOBJS:.o=.o.d)
PROGDEPS = $(LIBOBJS:.o=.o.d)

PROG = $(PROGSRCS:%.cpp=../src/%)


all: $(LIBOBJS) $(PROGOBJS) $(PROG)



# Linking: for each executable, link its o file together with all the non-executable o files
#linking commond have to be as below in albot2. Not like cogMap(mhossain) project makefile.
../src/%: $(LIBOBJS) $(PROGOBJS)
	@echo "---------------------------------------------------------------"
	@echo Building $*
	@echo " "
	
	$(CXX) $(LDFLAGS)  -o ../src/$* ../obj/$*.o $(LIBOBJS) $(LIBS)
	@echo " "
	@echo "		!!DONE!!			"
	@echo " "

# Compiling: first make dependencies, then compile all cpp files
../obj/%.o: %.cpp
	@echo "---------------------------------------------------------------"
	@echo Compiling $<
	@echo " "
	$(CXX) $(CPPFLAGS) $< -MM -MT $@ -MF $@.d
	$(CXX) $(CPPFLAGS) $< -o $@


clean:
	rm -f $(LIBOBJS) $(PROGOBJS) $(LIBDEPS) $(PROGDEPS) $(PROG)
cleanoutputs:
	rm -f ../outputs/Views/*
	rm -f ../outputs/Maps/*
	rm -f ../outputs/localSpaces/*
	
cleanmaps:
	rm -f ../outputs/Maps/*
	
cleansurfaces:
	rm -f ../outputs/surfaces/*

cleanpointclouds:
	rm -f ../outputs/pointCloud/*
cleanimages:
	rm -f ../outputs/raw/*
	rm -f ../outputs/rectified/*
	rm -f ../outputs/depth/*
	
cleaninputs:
	rm -f ../inputs/raw/*
	rm -f ../inputs/rectified/*
	rm -f ../inputs/depth/*
	rm -f ../inputs/color/*
	rm -f ../inputs/pointCloud/*
	rm -f ../inputs/surfaces/*



# Include the dependencies so the cpp files are recompiled if headers change
-include $(LIBDEPS)
-include $(PROGDEPS)

#####################################################################
#
# $Authors: Md Zulfikar Hossain and Martin Schrimpf (source: Guillaume Diallo-Mulliez)
# $Date: 28/08/2014
#
#####################################################################
