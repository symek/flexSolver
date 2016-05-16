g++  src/fur.cpp -o src/fur.o -c -Wall -I$CUDA_HOME/include -I/home/symek/work/flex/include \
-I/home/symek/work/flex/core -O3 -fPIC -ffast-math -fpermissive -fno-strict-aliasing 


g++ -o fur -fPIC -I$CUDA_HOME/include -I/home/symek/work/flex/include -I/home/symek/work/flex/core \
 -L$CUDA_HOME/lib64 -ldl -lcudart /home/symek/work/flex/core/maths.o src/fur.o /home/symek/work/flex/lib/linux64/flexRelease_x64.a
 