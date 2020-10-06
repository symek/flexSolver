 # g++ -DUT_DSO_TAGINFO='"3262197cbf01501926ad2d8c42369db22741d6f1c326d6a932ff93180914bfe618912890d67e2c704309e3b267da45541eee9851b6c2e85d530728e266f39bb843be57b0ee846394a3268d0847ba11fce917cff0cff18069f6bd37"'\
 #  -D_GNU_SOURCE -DLINUX -DAMD64 -m64 -fPIC -DSIZEOF_VOID_P=8 -DFBX_ENABLED=1 -DOPENCL_ENABLED=1 \
 # -DOPENVDB_ENABLED=1 -DSESI_LITTLE_ENDIAN -DENABLE_THREADS -DUSE_PTHREADS -D_REENTRANT -D_FILE_OFFSET_BITS=64 -c \
 # -I/home/symek/work/flex/core -I$CUDA_HOME/include -I/home/symek/work/flex/include  -DGCC4 -DGCC3 -Wno-deprecated \
 # -std=c++11 -isystem $HT/include -Wall -W -Wno-parentheses -Wno-sign-compare -Wno-reorder -Wno-uninitialized -Wunused \
 # -Wno-unused-parameter -Wno-unused-local-typedefs -O2 -fno-strict-aliasing  -o src/SOP_FlexWires.o src/SOP_FlexWires.cpp
g++ -DUT_DSO_TAGINFO='"3262197cbf01501926ad2d8c42369db22741d6f1c326d6a932ff93180914bfe618912890d67e2c704309e3b267da45541eee9851b6c2e85d530728e265ff9bbd48be57b0ee846394a3268d0847ba11fce917cff0cff18069f6bd37"'  \
-DVERSION=$HOUDINI_VERSION -D_GNU_SOURCE -DLINUX -DAMD64 -m64 -fPIC -DSIZEOF_VOID_P=8 -DFBX_ENABLED=1 -DOPENCL_ENABLED=1 \
-DOPENVDB_ENABLED=1 -DSESI_LITTLE_ENDIAN -DENABLE_THREADS -DUSE_PTHREADS -DDEBUG -D_REENTRANT -D_FILE_OFFSET_BITS=64 \
-c -I$CUDA_HOME/include -I/home/symek/work/flex/include  -DGCC4 -DGCC3 -Wno-deprecated -std=c++11 \
-isystem $HT/include -Wall -W -Wno-parentheses -Wno-sign-compare -Wno-reorder -Wno-uninitialized \
-Wunused -Wno-unused-parameter -Wno-unused-local-typedefs -O2 -fno-strict-aliasing  -g -DUT_ASSERT_LEVEL=2 \
-DMAKING_DSO -o src/SOP_FlexWires.o src/SOP_FlexWires.cpp

 g++ -shared src/SOP_FlexWires.o -lpthread -o ./SOP_FlexWires.so -L $HFS/dsolib -lHoudiniUI -lHoudiniOPZ -lHoudiniOP3 -lHoudiniOP2 \
 -lHoudiniOP1 -lHoudiniSIM -lHoudiniGEO -lHoudiniPRM -lHoudiniUT -lboost_system -L/usr/X11R6/lib64 -L/usr/X11R6/lib \
 -I/home/symek/work/flex/core -I$CUDA_HOME/include -I/home/symek/work/flex/include -L$CUDA_HOME/lib64 -ldl -lcudart \
 -lGLU -lGL -lX11 -lXext -lXi -ldl -Wl,-rpath,$HFS/dsolib /home/symek/work/flex/core/maths.o  \
 /home/symek/work/flex/lib/linux64/flexRelease_x64.a

mv SOP_FlexWires.so $HIH/dso

# g++  src/fur.cpp -o src/fur.o -c -Wall -std=c++11 -I$CUDA_HOME/include -I/home/symek/work/flex/include \
# -I/home/symek/work/flex/core -O3 -fPIC -ffast-math -fpermissive -fno-strict-aliasing 


# g++ -o fur -fPIC -fpermissive -I$CUDA_HOME/include -I/home/symek/work/flex/include -I/home/symek/work/flex/core \
#  -L$CUDA_HOME/lib64 -ldl -lcudart /home/symek/work/flex/core/maths.o src/fur.o /home/symek/work/flex/lib/linux64/flexRelease_x64.a
