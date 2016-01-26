SHELL = /bin/sh
CC = g++
AR = ar

CFG = release 

CUDA = $CUDA_HOME
NVCC = $(CUDA)/bin/nvcc 
CUDA_INC = $(CUDA)/include
CUDA_LIB = $(CUDA)/lib64

# CORE = $(ROOT)/core
# SRC = $(ROOT)/src
# LIB = $(ROOT)/lib/linux64
# DEMO = $(ROOT)/demo
# EXT = $(ROOT)/extensions
# INCLUDE = $(ROOT)/include
# EXTERNAL = $(ROOT)/external
# BIN = $(ROOT)/bin/linux32
# CUB = $(EXTERNAL)/cub-1.3.2	

CFLAGS.release = -g -Wall -I$(CUDA_INC) -I$(ROOT) -I$(INCLUDE) -O3 -fPIC -ffast-math -fpermissive -fno-strict-aliasing
CFLAGS.debug = -g -Wall -I$(CUDA_INC) -I$(ROOT) -I$(INCLUDE) -O0 -fPIC -fpermissive -fno-strict-aliasing
CFLAGS = $(CFLAGS.$(CFG))

CUFLAGS = -g -arch=sm_30 -m64 -Xcompiler -fPIC


LDFLAGS.release = -g -L/usr/lib -L$(CUDA_LIB) $(LIB)/flexRelease_x64.a -lGL -lglut -lGLU -lGLEW -lcudart
LDFLAGS.debug = -g -L/usr/lib -L$(CUDA_LIB) $(LIB)/flexDebug_x64.a -lGL -lglut -lGLU -lGLEW -lcudart
LDFLAGS = $(LDFLAGS.$(CFG))

TARGET.release = $(ROOT)/bin/linux64/flexDemoRelease 
TARGET.debug = $(ROOT)/bin/linux64/flexDemoDebug 
TARGET = $(TARGET.$(CFG))

SOURCES = $(wildcard *.cpp) $(CORE)/platform.cpp $(CORE)/perlin.cpp $(CORE)/sdf.cpp $(CORE)/shader.cpp $(CORE)/maths.cpp $(CORE)/mesh.cpp $(CORE)/aabbtree.cpp $(CORE)/tga.cpp $(CORE)/pfm.cpp $(CORE)/voxelize.cpp $(DEMO)/main.cpp $(DEMO)/shaders.cpp $(DEMO)/imgui.cpp $(DEMO)/imguiRenderGL.cpp
HEADERS = $(wildcard *.h) $(wildcard $(DEMO)/*.h)
OBJECTS = $(SOURCES:.cpp=.o)

all: $(TARGET)

release:
	$(MAKE) $(MAKEFILE) CFG=release

debug:
	$(MAKE) $(MAKEFILE) CFG=debug

$(TARGET): $(OBJECTS) makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $(TARGET) 

clean:
	-rm -f $(OBJECTS) $(TARGET)

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET)

.PHONY : all clean

