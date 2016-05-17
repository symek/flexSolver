#include <flex.h>
#include <maths.h>
#include <memory>
#include <iostream>
// #include <thread>
// #include <chrono>
#include "fur.hpp"
#include <GU/GU_Detail.h>
// #include <mutex>



int interpretError(const FlexError error)
{
     switch(error)
    {
        case eFlexErrorNone:
            return 0;
            case eFlexErrorWrongVersion:
            std::cout << "FlexError: The header version does not match the library binary." << std::endl;
            return 1;
        case eFlexErrorInsufficientGPU:
            std::cout << "FlexError:The GPU associated with the calling thread does not meet requirements." << std::endl;
            std::cout << "An SM3.0 GPU or above is required" << std::endl;
            return 2;
        case eFlexErrorDriverFailure:
            std::cout << "GPU driver is too old." << std::endl;
            return 3;
    }
    return 4;
}

void copyPointAttribs(const GU_Detail &gdp, const bool roots, 
                     float* particles_ptr, float* velocities_ptr)
{
    GA_ROHandleV3  vel_handle(&gdp, GA_ATTRIB_POINT, "v");
    GA_ROHandleI   root_handle(&gdp, GA_ATTRIB_POINT, "root");
    const int      vel_valid = vel_handle.isValid();
    const int      root_valid = root_handle.isValid();

    GA_Offset ptoff;
    int rootValue = 1;
    GA_FOR_ALL_PTOFF(&gdp, ptoff) {
        if (root_valid && roots) {
            rootValue = root_handle.get(ptoff);
        }
        const int offset = static_cast<const int>(ptoff);
        if (rootValue) {
            const UT_Vector3 pos = gdp.getPos3(ptoff);
            particles_ptr[4*offset]   = pos.x();
            particles_ptr[4*offset+1] = pos.y();
            particles_ptr[4*offset+2] = pos.z();
            particles_ptr[4*offset+3] = .1;
        }
        if (vel_valid) {
            const UT_Vector3 vel    = vel_handle.get(ptoff);
            velocities_ptr[3*offset]   += vel.x(); // We better initialize velocities...
            velocities_ptr[3*offset+1] += vel.y();
            velocities_ptr[3*offset+2] += vel.z();
        }
    }  
}


void copySpringAttribs(const GU_Detail &gdp,  int   *springIndices,
                       float *springLengths,  float *springCoefficients)
{
    uint springCounter = 0;
    GA_ROHandleF     stiffness_handle(&gdp, GA_ATTRIB_POINT, "stiffness");
    for (GA_Iterator it(gdp.getPrimitiveRange()); !it.atEnd(); ++it)
    {
        const GEO_Primitive *prim = gdp.getGEOPrimitive(*it);
        const GA_Range vertices = prim->getVertexRange();
        GA_Offset vi;
        for (vi = GA_Offset(1); vi < vertices.getEntries(); ++vi) {           
            const GA_Offset vtx1 = prim->getVertexOffset(vi-1);
            const GA_Offset vtx2 = prim->getVertexOffset(vi);
            springIndices[2*springCounter]     = static_cast<int>(vtx1);
            springIndices[(2*springCounter)+1] = static_cast<int>(vtx2);
            const UT_Vector3 vp1 = gdp.getPos3(vtx1);
            const UT_Vector3 vp2 = gdp.getPos3(vtx2);
            springLengths[springCounter] = UT_Vector3(vp2 - vp1).length();
            if (stiffness_handle.isValid()) {
                const float stiffness1 = stiffness_handle.get(vtx1);
                const float stiffness2 = stiffness_handle.get(vtx2);
                springCoefficients[springCounter] = stiffness1*stiffness2;
            }
            else {
                springCoefficients[springCounter] = .5f;
            }
            springCounter++;
        }
    }

}

void copyMesh(const GU_Detail &gdp, float *points, int *vertices, \
            int &npoints, int &npoly, float *lower, float *upper)
{
    UT_BoundingBox bbox(0,0,0,1,1,1);
    if (gdp.getBBox(&bbox)) {
        lower[0] = bbox.xmin(); lower[1] = bbox.ymin(); 
        lower[2] = bbox.zmin(); upper[0] = bbox.xmin(); 
        upper[1] = bbox.ymin(); upper[2] = bbox.zmin();
    }
    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(&gdp, ptoff) {
        const UT_Vector3 pos  = gdp.getPos3(ptoff);
        const int offset = static_cast<int>(ptoff);
        points[3*offset]   = pos.x(); 
        points[3*offset+1] = pos.y(); 
        points[3*offset+2] = pos.z(); 
    }
    npoints = gdp.getNumPoints();
    npoly   = gdp.getNumPrimitives();  
    int vertex_index = 0;
    for (GA_Iterator it(gdp.getPrimitiveRange()); !it.atEnd(); ++it) {
        const GEO_Primitive *prim = gdp.getGEOPrimitive(*it);
        GA_Primitive::const_iterator vt;
        for (prim->beginVertex(vt); !vt.atEnd(); ++vt) {
            const GA_Offset voff = vt.getVertexOffset();
            vertices[vertex_index] = static_cast<int>(voff);
            vertex_index++;
        }
    }
}

uint getNumSprings(const GU_Detail &gdp)
{
    uint springs = 0;
    for (GA_Iterator it(gdp.getPrimitiveRange()); !it.atEnd(); ++it) {
        const GEO_Primitive *prim = gdp.getGEOPrimitive(*it);
        const GA_Range vertices = prim->getVertexRange();
        springs += vertices.getEntries() - 1; }
    return springs;
}


void saveGeometry(const GU_Detail &gdp, const int frame)
{
    const char* tmp = "./tmp/particles.%i.bgeo";
    int sz = std::snprintf(NULL, 0, tmp, frame);
    char filename[sz+1]; // note +1 for null terminator
    std::snprintf(filename, sz+1, tmp, frame);
    // std::cout << "Writing to: " << filename << std::endl;
    gdp.save(filename, 0, 0);

}
int main(void)
{
    /* init and catch errors*/
    FlexError status = flexInit();
    const int error = interpretError(status);
    if (error)
        return error;


    GU_Detail gdp;
    GU_Detail collisionGeo;
    gdp.load("./fur.bgeo");
    collisionGeo.load("./sphere.bgeo");
    collisionGeo.convex(); // only triangles
    std::cout << "Setting max particles: " << gdp.getNumPoints() <<std::endl;

    const int maxParticles = gdp.getNumPoints();// + collisionGeo.getNumPoints();
    const int maxDiffuse = 0;

    FlexSolver* solver = flexCreateSolver(maxParticles, maxDiffuse);
    FlexTimers timer   = FlexTimers();
    FlexParams parms   = FlexParams();
    parms.mPlanes[0][0] = 0.0f;
    parms.mPlanes[0][1] = 1.0f;
    parms.mPlanes[0][2] = 0.0f;
    parms.mPlanes[0][3] = 0.0f;

    parms.mNumPlanes = 1;

    // set Fluid like parms:
    InitFlexParams(parms);
    flexSetParams(solver, &parms);

    // alloc CUDA pinned host memory to allow asynchronous memory transfers
    Vec4*  particles      = reinterpret_cast<Vec4*>(flexAlloc(maxParticles*sizeof(Vec4)));
    Vec3*  velocities     = reinterpret_cast<Vec3*>(flexAlloc(maxParticles*sizeof(Vec3)));
    float* particles_ptr  = reinterpret_cast<float*>(particles);
    float* velocities_ptr = reinterpret_cast<float*>(velocities);
    int*   actives        = reinterpret_cast<int*>(flexAlloc(sizeof(int)*maxParticles));
    for (int i=0; i < maxParticles; ++i) {
        actives[i] = i;
        velocities_ptr[3*i]   = 0.0;
        velocities_ptr[3*i+1] = 0.0;
        velocities_ptr[3*i+2] = 0.0;
    }

    // set initial particle data
    copyPointAttribs(gdp, false, particles_ptr, velocities_ptr);


    // Springs setup:
    uint  numSprings     = getNumSprings(gdp);
    std::cout << "Springs created      : " <<  numSprings <<std::endl;
    int   *springIndices = reinterpret_cast<int*>(flexAlloc(sizeof(int)*2*numSprings));
    float *springLengths = reinterpret_cast<float*>(flexAlloc(sizeof(float)*numSprings));
    float *springCoefficients =reinterpret_cast<float*>(flexAlloc(sizeof(float)*numSprings));
    copySpringAttribs(gdp, springIndices, springLengths, springCoefficients);

    flexSetActive(solver, actives, maxParticles, eFlexMemoryHostAsync);
    flexSetParticles(solver, particles_ptr, maxParticles, eFlexMemoryHostAsync);
    flexSetVelocities(solver, velocities_ptr, maxParticles, eFlexMemoryHostAsync);
    flexSetSprings(solver, springIndices, springLengths, springCoefficients, \
                numSprings, eFlexMemoryHostAsync);


    /*------------- Collision objects ------------- */

    std::vector<float> meshPoints;
    std::vector<int>   meshIndices;
    int npoints = 0; int npoly = 0;
    float lower[3]; float upper[3];
    meshPoints.resize(collisionGeo.getNumPoints()*3);
    meshIndices.resize(collisionGeo.getNumPrimitives()*3);

    copyMesh(collisionGeo, &meshPoints[0], &meshIndices[0], npoints, \
        npoly, &lower[0], &upper[0]);

    FlexTriangleMesh *mesh = flexCreateTriangleMesh();
    FlexCollisionGeometry collisionGeometry;
    collisionGeometry.mTriMesh.mMesh  = mesh;
    collisionGeometry.mTriMesh.mScale = 1.0f;

    flexUpdateTriangleMesh(mesh, &meshPoints[0], &meshIndices[0], npoints, \
        npoly, &lower[0], &upper[0], eFlexMemoryHostAsync);
    
    std::vector<FlexCollisionGeometry> collisionGeometries;
    collisionGeometries.push_back(collisionGeometry);

    const int numGeometryEntries = 1; 
    float shapeAabbMaxs[4] = {0.5, 2.5, 0.5, 0};
    float shapeAabbMins[4] = {-0.5, 1.5, -0.5, 0};
    int shapeOffsets[1]; shapeOffsets[0] = 0;
    float shapePositions[4] = {0,0,0,0};
    float shapeRotations[4] = {1,0,0,0};
    float shapePrevPositions[4] = {0,0,0,0};
    float shapePrevRotations[4] = {1,0,0,0};
    int shapeFlags[1]; 
    shapeFlags[0] = flexMakeShapeFlags(eFlexShapeTriangleMesh, false);
    int numShapes = 1;
    flexSetShapes(solver, &collisionGeometries[0], numGeometryEntries, shapeAabbMins, shapeAabbMaxs, 
        shapeOffsets, shapePositions, shapeRotations, shapePrevPositions, shapePrevRotations, \
            shapeFlags, numShapes, eFlexMemoryHostAsync);

   

  
    GU_Detail source;
    source.copy(gdp);

    int counter = 0;
    while (counter < 240)
    {
        const float dt = 1.0f/24.0f;

        // update positions, apply custom force fields, etc
        // ModifyParticles(particles, velocities);
        copyPointAttribs(source, true, particles_ptr, velocities_ptr);
        shapePositions[0]     = SYSsin(counter/24.0);
        shapePrevPositions[0] = SYSsin(counter-1/24.0);

        // update GPU data asynchronously

        flexSetShapes(solver,  &collisionGeometries[0], numGeometryEntries, shapeAabbMins, shapeAabbMaxs, 
        shapeOffsets, shapePositions, shapeRotations, shapePrevPositions,\
        shapePrevRotations, shapeFlags, numShapes, eFlexMemoryHostAsync);
    
        flexSetParticles(solver, particles_ptr, maxParticles, eFlexMemoryHostAsync);
        flexSetVelocities(solver, velocities_ptr, maxParticles, eFlexMemoryHostAsync);
        flexSetSprings(solver, springIndices, springLengths, springCoefficients, \
            numSprings, eFlexMemoryHostAsync);
        

        // tick solver
        flexUpdateSolver(solver, dt, 1, &timer);

        // kick off async memory reads from device
        flexGetParticles(solver, particles_ptr, maxParticles, eFlexMemoryHostAsync);
        // flexGetVelocities(solver, velocities_ptr, maxParticles, eFlexMemoryHostAsync);

        // wait for GPU to finish working (can perform async. CPU work here)
        flexSetFence();
        flexWaitFence();

        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(&gdp, ptoff) {
            const int offset = static_cast<int>(ptoff)*4;
            float *point     = &particles_ptr[offset];
            const UT_Vector3 pos(point[0], point[1], point[2]);
            gdp.setPos3(ptoff, pos);
        }
        // updated particle data is ready to be used
        saveGeometry(gdp, counter);
        counter++;

            
    }

    std::cout << "Total time: " <<  timer.mTotal << std::endl;
    std::cout << "Density ca: " <<  timer.mCalculateDensity << std::endl;
    std::cout << "Collisions: " <<  timer.mCollideParticles << std::endl;

    flexFree(particles);
    flexFree(velocities);
    flexFree(springIndices);
    flexFree(springLengths);
    flexFree(springCoefficients);
    // flexFree(phases);
    // flexDestroyTriangleMesh(mesh);
    flexDestroySolver(solver);
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
//  GU_Detail gdp;
//  gdp.load("./furs.bgeo");
//  std::cout << gdp.getNumPoints() <<std::endl;

//  FlexError status = flexInit();
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




//  return 0;
// }