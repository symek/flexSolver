g++ src/solver.cpp -o solver -g -Wall -I$CUDA_HOME/include -I/home/symek/work/flex/include \
-O3 -fPIC -ffast-math -fpermissive -fno-strict-aliasing -L$CUDA_HOME/lib64 -lcudart \
/home/symek/work/flex/lib/linux64/flexRelease_x64.a 