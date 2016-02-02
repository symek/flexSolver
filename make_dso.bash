 hcustom -I/home/symek/work/flex/include -I${CUDA_HOME}/include -L${CUDA_HOME}/lib64 \
 -L/home/symek/work/flex/lib/linux64 -lcudart -lflexRelease_x64  src/SOP_FlexSolver.cpp
