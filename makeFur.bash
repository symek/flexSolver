g++ -DUT_DSO_TAGINFO='"3262197cbf01501926ad2d8f553b97f5204f86a5812acca932f8d0594541af8f349365de812c43570c0bfde12b9f751c04a0ef10efafb60a537b7ca177fb97ac4aa65fb3e9885a87a3349d19519011fcab4ed5f0cfa2d966eaec1d580413ac49f034"'  \
-DVERSION=\"15.0.459\" -D_GNU_SOURCE -DLINUX -DAMD64 -m64 -fPIC -DSIZEOF_VOID_P=8 -DFBX_ENABLED=1 \
-DOPENCL_ENABLED=1 -DOPENVDB_ENABLED=1 -DSESI_LITTLE_ENDIAN -DENABLE_THREADS -DUSE_PTHREADS -D_REENTRANT \
-D_FILE_OFFSET_BITS=64 -c -I/opt/package/rez_packages/cuda/7.5.18/platform-linux/arch-x86_64/os-CentOS_Linux-7/include \
-I/home/symek/work/flex/include  -DGCC4 -DGCC3 -Wno-deprecated -std=c++11 \
-isystem /opt/package/rez_packages/houdini/15.0.459/platform-linux/arch-x86_64/os-CentOS_Linux-7/toolkit/include \
-Wall -W -Wno-parentheses -Wno-sign-compare -Wno-reorder -Wno-uninitialized -Wunused -Wno-unused-parameter \
-Wno-unused-local-typedefs -O2 -fno-strict-aliasing  -o src/fur.o src/fur.cpp


g++ src/fur.o -lpthread -o ./fur -L /opt/package/rez_packages/houdini/15.0.459/platform-linux/arch-x86_64/os-CentOS_Linux-7/dsolib \
-lHoudiniUI -lHoudiniOPZ -lHoudiniOP3 -lHoudiniOP2 -lHoudiniOP1 -lHoudiniSIM -lHoudiniGEO -lHoudiniPRM -lHoudiniUT -lboost_system \
-L/usr/X11R6/lib64 -L/usr/X11R6/lib -I/opt/package/rez_packages/cuda/7.5.18/platform-linux/arch-x86_64/os-CentOS_Linux-7/include \
-I/home/symek/work/flex/include -lGLU -lGL -lX11 -lXext -lXi -ldl -Wl,-rpath,/opt/package/rez_packages/houdini/15.0.459/platform-linux/arch-x86_64/os-CentOS_Linux-7/dsolib \
-L$CUDA_HOME/lib64 -lcudart /home/symek/work/flex/lib/linux64/flexRelease_x64.a 