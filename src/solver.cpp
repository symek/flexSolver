#include <flex.h>
#include <stddef.h>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <vector>
#include <math.h> // floor()
#include <time.h>

#include <stdlib.h> // for NULL wtf?


const int maxParticles = 65536/10;
const int maxDiffuse = 0;

struct Vec4
{
    float x;
    float y;
    float z;
    float w;
};

struct Vec3
{
    float x;
    float y;
    float z;
};



void SetFlexParams(FlexParams &g_params)
{
    g_params.mGravity[0] = 0.0f;
    g_params.mGravity[1] = -9.8f;
    g_params.mGravity[2] = 0.0f;

    g_params.mWind[0] = 0.1f;
    g_params.mWind[1] = 0.0f;
    g_params.mWind[2] = 0.02f;

    g_params.mRadius = 0.15f;
    g_params.mViscosity = 0.0f;
    g_params.mDynamicFriction = 0.0f;
    g_params.mStaticFriction = 0.0f;
    g_params.mParticleFriction = 0.0f; // scale friction between particles by default
    g_params.mFreeSurfaceDrag = 0.0f;
    g_params.mDrag = 0.0f;
    g_params.mLift = 0.0f;
    g_params.mNumIterations = 4;
    g_params.mFluidRestDistance = 0.0f;
    g_params.mSolidRestDistance = 0.0f;
    g_params.mAnisotropyScale = 1.0f;
    g_params.mDissipation = 0.0f;
    g_params.mDamping = 0.0f;
    g_params.mParticleCollisionMargin = 0.0f;
    g_params.mShapeCollisionMargin = 0.0f;
    g_params.mCollisionDistance = 0.0f;
    g_params.mPlasticThreshold = 0.0f;
    g_params.mPlasticCreep = 0.0f;
    g_params.mFluid = false;
    g_params.mSleepThreshold = 0.0f;
    g_params.mShockPropagation = 0.0f;
    g_params.mRestitution = 0.0f;
    g_params.mSmoothing = 1.0f;
    g_params.mMaxVelocity = 1000.0;
    g_params.mRelaxationMode = eFlexRelaxationLocal;
    g_params.mRelaxationFactor = 1.0f;
    g_params.mSolidPressure = 1.0f;
    g_params.mAdhesion = 0.0f;
    g_params.mCohesion = 0.025f;
    g_params.mSurfaceTension = 0.0f;
    g_params.mVorticityConfinement = 0.0f;
    g_params.mBuoyancy = 1.0f;
    g_params.mDiffuseThreshold = 100.0f;
    g_params.mDiffuseBuoyancy = 1.0f;
    g_params.mDiffuseDrag = 0.8f;
    g_params.mDiffuseBallistic = 16;
    g_params.mDiffuseSortAxis[0] = 0.0f;
    g_params.mDiffuseSortAxis[1] = 0.0f;
    g_params.mDiffuseSortAxis[2] = 0.0f;
    g_params.mEnableCCD = false;

    // planes created after particles
    g_params.mNumPlanes = 1;

    float radius = 0.1f;
    g_params.mRadius = radius;
    g_params.mDynamicFriction = 0.1f;
    g_params.mFluid = true;
    g_params.mViscosity = 0.0f;     
    g_params.mNumIterations = 5;
    g_params.mVorticityConfinement = 0.0f;
    g_params.mAnisotropyScale = 25.0f;
    g_params.mFluidRestDistance = g_params.mRadius*0.55f;


    // by default solid particles use the maximum radius
    if (g_params.mFluid && g_params.mSolidRestDistance == 0.0f)
        g_params.mSolidRestDistance = g_params.mFluidRestDistance;
    else
        g_params.mSolidRestDistance = g_params.mRadius;

    // collision distance with shapes half the radius
    if (g_params.mCollisionDistance == 0.0f)
    {
        g_params.mCollisionDistance = g_params.mRadius*0.5f;

        if (g_params.mFluid)
            g_params.mCollisionDistance = g_params.mFluidRestDistance*0.5f;
    }

    // default particle friction to 10% of shape friction
    if (g_params.mParticleFriction == 0.0f)
        g_params.mParticleFriction = g_params.mDynamicFriction*0.1f; 

    // add a margin for detecting contacts between particles and shapes
    if (g_params.mShapeCollisionMargin == 0.0f)
        g_params.mShapeCollisionMargin = g_params.mCollisionDistance*0.25f;
}

float randZeroToOne()
{
    return rand() / (RAND_MAX + 1.);
}

float rand_FloatRange(float a, float b)
{
    return ((b-a)*((float)rand()/RAND_MAX))+a;
}


void InitPositions(Vec4 *particles)
{
    srand (time(NULL));
    for (int i = 0; i < maxParticles; ++i)
    {
        particles[i].x = rand_FloatRange(-2.0f, 2.0f);
        particles[i].y = rand_FloatRange(-2.0f, 2.0f);
        particles[i].z = rand_FloatRange(-2.0f, 2.0f);
        particles[i].w = .1f;
    }
}

void InitVelocities(Vec3 *velocities)
{
    srand(time(NULL));
    for (int i = 0; i < maxParticles; ++i)
    {
        velocities[i].x = rand_FloatRange(-1.0f, 1.0f);
        velocities[i].y = rand_FloatRange(-1.0f, 1.0f);
        velocities[i].z = rand_FloatRange(-1.0f, 1.0f);
    }
}

void InitPhases(int *phases)
{
    for (int i = 0; i < maxParticles; ++i)
        phases[i] = flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid);
}

void InitParticles(Vec4 *particles, Vec3 *velocities, int *phases)
{
  InitPositions(particles);
  InitVelocities(velocities);
  InitPhases(phases);
}

void RenderParticles(float *particles, int frame)
{
    const char* tmp = "./tmp/particles.%i.obj";
    int sz = std::snprintf(NULL, 0, tmp, frame);
    char filename[sz+1]; // note +1 for null terminator
    std::snprintf(filename, sz+1, tmp, frame);
    std::cout << "Writing to: " << filename << std::endl;
    std::ofstream objfile;
    objfile.open(filename);
    for (int i=0; i< maxParticles; ++i)
    {
        objfile << "v ";
        objfile << particles[(i*3)]    << " ";
        objfile << particles[(i*3)+1]  << " ";
        objfile << particles[(i*3)+2]  << " ";
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
                std::cout << "FlexError:The GPU associated with the calling thread does not meet requirements. An SM3.0 GPU or above is required" << std::endl;
                return 1;
            }
        }
        FlexSolver* solver = flexCreateSolver(maxParticles, maxDiffuse);
        FlexTimers timer   = FlexTimers();
        FlexParams parms   = FlexParams();

        // set Fluid like parms:
        SetFlexParams(parms);
        flexSetParams(solver, &parms);

        // std::cout << parms.mGravity[0] << parms.mGravity[1] << parms.mGravity[2] << std::endl;

        // alloc CUDA pinned host memory to allow asynchronous memory transfers
        Vec4* particles  = static_cast<Vec4*>(flexAlloc(maxParticles*sizeof(Vec4)));
        Vec3* velocities = static_cast<Vec3*>(flexAlloc(maxParticles*sizeof(Vec3)));
        float *output    = new float[maxParticles*4];
        int* phases      = static_cast<int*>(flexAlloc(maxParticles*sizeof(int)));
        int* actives     = static_cast<int*>(flexAlloc(maxParticles*sizeof(int)));
        
        for (int i=0; i < maxParticles; ++i)
            actives[i] = i;

        // float * particlesPtr  = (float*)particles;
        // float * velocitiesPtr = (float*)velocities;

        // // set initial particle data
        InitParticles(particles, velocities, phases);
        float t              = 0;
        const int   fps      = 24;
        const int   substeps = 10;
        const float seconds  = 5;

        flexSetParticles(solver, (float*)&particles[0], maxParticles, eFlexMemoryHost);
        flexSetVelocities(solver, (float*)&velocities[0], maxParticles, eFlexMemoryHost);
        flexSetPhases(solver, &phases[0], maxParticles, eFlexMemoryHost);
        flexSetActive(solver, &actives[0], maxParticles, eFlexMemoryHost);
        std::cout << "Actives: " << flexGetActiveCount(solver) << std::endl;
        while (t <= seconds)
        {
                const float dt = 1.0f/(fps);

                // update positions, apply custom force fields, etc
                // ModifyParticles(particles, velocities);

                // tick solver
                flexUpdateSolver(solver, dt, substeps, &timer);
                // update GPU data asynchronously
                // flexSetParticles(solver, particlesPtr, maxParticles, eFlexMemoryHost);
                InitVelocities(velocities);
                flexSetVelocities(solver, (float*)&velocities[0], maxParticles, eFlexMemoryHost);

                t += dt;

                // // wait for GPU to finish working (can perform async. CPU work here)
                // // kick off async memory reads from device
                int part = 0;
                std::cout << "Before update: ";
                std::cout << particles[part].x << ", " << particles[part].y << ", " << particles[part].z;
                std::cout << std::endl;
                flexGetParticles(solver, (float*)&output[0], maxParticles, eFlexMemoryHost);
                // flexSetFence();
                // flexGetVelocities(solver, velocitiesPtr, maxParticles, eFlexMemoryHost);
                std::cout << "After  update: ";
                std::cout << output[part] << ", " << output[part+1] << ", " << output[part+2];
                std::cout << std::endl;
                // std::cout << velocities[part].x << ", " << velocities[part].y << ", " << velocities[part].z;
                // std::cout << std::endl;


                const int frame =  t / dt;
                RenderParticles(output, frame);
                // flexWaitFence();

        }

        // std::cout << "Counter: " << counter << std::endl;
        // std::cout << std::endl;
        std::cout << "Total time: " <<  timer.mTotal << std::endl;
        std::cout << "Density ca: " <<  timer.mCalculateDensity << std::endl;
        std::cout << "Collisions: " <<  timer.mCollideParticles << std::endl;
        
        // for (int i = 0; i < 5; ++i)
        //     std::cout << particles[i].x << ", " << particles[i].y << ", " << particles[i].z;
        // std::cout << std::endl;

        flexFree(particles);
        flexFree(velocities);
        flexFree(phases);
        flexFree(actives);
        delete [] output;

        flexDestroySolver(solver);
        flexShutdown();

        return 0;
}