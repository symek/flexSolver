#include <flex.h>
#include <maths.h>

const int maxParticles = 65536;
const int maxDiffuse = 0;

int main(void)
{
        flexInit();
        // flexSolver* solver = flexCreateSolver(maxParticles, maxDiffuse);

        // // alloc CUDA pinned host memory to allow asynchronous memory transfers
        // Vec4* particles = flexAlloc(maxParticles*sizeof(Vec4));
        // Vec3* velocities = flexAlloc(maxParticles*sizeof(Vec3));
        // int* phases = flexAlloc(maxParticles*sizeof(int));

        // // set initial particle data
        // InitParticles(particles, velocities, phases);

        // while (!finished)
        // {
        //         const float dt = 1.0f/60.0f;

        //         // update positions, apply custom force fields, etc
        //         ModifyParticles(particles, velocities);

        //         // update GPU data asynchronously
        //         flexSetParticles(solver, particles, maxParticles, eFlexMemoryHostAsync);
        //         flexSetVelocities(solver, velocities, maxParticles, eFlexMemoryHostAsync);

        //         // tick solver
        //         flexUpdateSolver(solver, dt, 1);

        //         // kick off async memory reads from device
        //         flexGetParticles(solver, particles, maxParticles, eFlexMemoryHostAsync);
        //         flexGetVelocities(solver, velocities, maxParticles, eFlexMemoryHostAsync);

        //         // wait for GPU to finish working (can perform async. CPU work here)
        //         flexSetFence();
        //         flexWaitFence();

        //         // updated particle data is ready to be used
        //         RenderParticles(particles, velocities);
        // }

        // flexFree(particles);
        // flexFree(velocities);
        // flexFree(phases);

        // flexDestroySolver(solver);
        flexShutdown();

        return 0;
}















// #include <flex.h>
// #include <stddef.h>
// #include <iostream>
// #include <cstdio>
// #include <fstream>
// #include <vector>
// #include <math.h> // floor()
// #include <time.h>
// #include <stdlib.h> // for NULL wtf?"


// #include "fur.hpp"

// #include <GU/GU_Detail.h>




// int main()
// {
// 	GU_Detail gdp;
// 	gdp.load("./furs.bgeo");
// 	std::cout << gdp.getNumPoints() <<std::endl;

// 	FlexError status = flexInit();
//     if(status)
//     {
//         switch(status)
//         {
//             case 0:
//             break;
//             case 1:
//             std::cout << "FlexError: The header version does not match the library binary." << std::endl;
//             return 1;
//             case 2:
//             std::cout << "FlexError:The GPU associated with the calling thread does not meet requirements." << std::endl;
//             std::cout << "An SM3.0 GPU or above is required" << std::endl;
//             return 1;
//         }
//     }




// 	return 0;
// }