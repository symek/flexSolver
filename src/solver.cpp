#include "solver.hpp"



void RenderParticles(float *particles, int frame)
{
    const char* tmp = "./tmp/particles.%i.obj";
    int sz = std::snprintf(NULL, 0, tmp, frame);
    char filename[sz+1]; // note +1 for null terminator
    std::snprintf(filename, sz+1, tmp, frame);
    std::cout << "Writing to: " << filename << std::endl;
    std::ofstream objfile;
    objfile.open(filename);
    for (int i=0; i< maxParticles; i+=4)
    {
        objfile << "v ";
        objfile << particles[i]    << " ";
        objfile << particles[i+1]  << " ";
        objfile << particles[i+2]  << " ";
        objfile << "\n";
    }

    objfile.close();
}

int main(void)
{
        FlexError status = flexInit();
        if(status)
        {
            switch(status)
            {
                case 0:
                break;
                case 1:
                std::cout << "FlexError: The header version does not match the library binary." << std::endl;
                return 1;
                case 2:
                std::cout << "FlexError:The GPU associated with the calling thread does not meet requirements." << std::endl;
                std::cout << "An SM3.0 GPU or above is required" << std::endl;
                return 1;
            }
        }
        FlexSolver* solver = flexCreateSolver(maxParticles, maxDiffuse);
        FlexTimers timer   = FlexTimers();
        FlexParams parms   = FlexParams();

        // Collision plane:
        // std::vector<float> ground(4, 0.0f);
        // ground[1] = 1.0f; 
        // float ground[4]; ground[0] = 0.0f;
        // ground[1] = 1.0f;ground[2] = 0.0f;
        // ground[3] = 0.0f;
        parms.mPlanes[0][0] = 0.0f;
        parms.mPlanes[0][1] = 1.0f;
        parms.mPlanes[0][2] = 0.0f;
        parms.mPlanes[0][3] = 0.0f;

        parms.mNumPlanes = 1;

        // set Fluid like parms:
        InitFlexParams(parms);
        flexSetParams(solver, &parms);

        // alloc CUDA pinned host memory to allow asynchronous memory transfers
        std::vector<float> particles(maxParticles*4, 0.0f);
        std::vector<float> velocities(maxParticles*3, 0.0f);
        std::vector<float> normals(maxParticles*4, 0.0f);
        std::vector<int>   actives(maxParticles, 0.0f);

        int fluidPhase    = flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid);
        std::vector<int>   phases(maxParticles, fluidPhase);

        // Create active particels set:
        for (int i=0; i < maxParticles; ++i)
            actives[i] = i;

        // set initial particle data
        InitPositions(&particles[0]);
        InitNormals(&normals[0]);
        InitVelocities(&velocities[0]);

        float t              = 0;
        const int   fps      = 24;
        const int   substeps = 1;
        const float seconds  = 5;
        flexSetParticles(solver, &particles[0], maxParticles, eFlexMemoryHost);
        flexSetVelocities(solver, &velocities[0], maxParticles, eFlexMemoryHost);
        flexSetNormals(solver, &normals[0], maxParticles, eFlexMemoryHost);
        flexSetPhases(solver, &phases[0], maxParticles, eFlexMemoryHost);
        flexSetActive(solver, &actives[0], maxParticles, eFlexMemoryHost);
       
        int counter = 0;
        while (t <= seconds)
        {
                const float dt = 1.0f/(fps);
                // flexSetParams(solver, &parms);

                // update positions, apply custom force fields, etc
                // ModifyParticles(particles, velocities);
                InitVelocities(&velocities[0]);
                flexSetVelocities(solver, &velocities[0], maxParticles, eFlexMemoryHost);

                // tick solver
                flexUpdateSolver(solver, dt, substeps, &timer);
                // update GPU data asynchronously

                t += dt;

                // // wait for GPU to finish working (can perform async. CPU work here)
                // // kick off async memory reads from device
                // int part = 1000;
              
                flexGetParticles(solver, (float*)&particles[0], maxParticles, eFlexMemoryHost);

    
                const int frame =  t / dt;
                RenderParticles(&particles[0], frame);
                counter++;

        }

        std::cout << "Total time: " <<  timer.mTotal << std::endl;
        std::cout << "Density ca: " <<  timer.mCalculateDensity << std::endl;
        std::cout << "Collisions: " <<  timer.mCollideParticles << std::endl;
        

        // flexFree(particles);
        // flexFree(velocities);
        // flexFree(phases);
        // flexFree(actives);
        // delete [] output;

        flexDestroySolver(solver);
        flexShutdown();

        return 0;
}