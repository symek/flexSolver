#include <flex.h>
#include <maths.h>
#include <memory>
#include <iostream>
// #include <thread>
// #include <chrono>
#include "fur.hpp"
#include <GU/GU_Detail.h>
// #include <mutex>

#define PIN_TO_ANIMATION 1

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

void copyPointAttribs(const GU_Detail &gdp, std::vector<float> &particles, 
                      std::vector<float> &velocities, std::vector<int> &actives, 
                      std::vector<int> &phases, const float glueToAnimation=0)
{
    GA_ROHandleV3  vel_handle(&gdp, GA_ATTRIB_POINT, "v");
    GA_ROHandleF  glue_handle(&gdp, GA_ATTRIB_POINT, "gluetoanimation");
    GA_ROHandleF  mass_handle(&gdp, GA_ATTRIB_POINT, "mass");
    GA_ROHandleI phase_handle(&gdp, GA_ATTRIB_POINT, "phase");
    
    const int   vel_valid = vel_handle.isValid();
          int  glue_valid = glue_handle.isValid();
    const int  mass_valid = mass_handle.isValid();
    const int phase_valid = phase_handle.isValid();

    float glueValue = 0.0f;
    float massValue = 1.0f;

    if (glueToAnimation < 0.0f) {
        glue_valid = 0;
        glueValue = 1.0f;
    }

    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(&gdp, ptoff) 
    {
        const int offset = static_cast<const int>(ptoff);

        if (glue_valid) {
            glueValue = glue_handle.get(ptoff);
            if (glueValue == 1.0 && glueToAnimation > 0)
                actives[offset] = -1;
        }

        if (mass_valid) {
            massValue = mass_handle.get(ptoff);
        }

        const UT_Vector3 pos = gdp.getPos3(ptoff);

        UT_Vector3 delta(particles[4*offset],
                         particles[4*offset+1],
                         particles[4*offset+2]);

        delta -= (delta - pos)*glueValue;
        particles[4*offset]   = delta.x();
        particles[4*offset+1] = delta.y();
        particles[4*offset+2] = delta.z();
        particles[4*offset+3] = massValue;
        
        // if (vel_valid && glueValue != 1.0f) {
        //     const UT_Vector3 vel    = vel_handle.get(ptoff);
        //     velocities[3*offset]   += vel.x();
        //     velocities[3*offset+1] += vel.y();
        //     velocities[3*offset+2] += vel.z();
        // }

        if (glueValue == 1.0f) {
            velocities[3*offset]   = 0.0f;
            velocities[3*offset+1] = 0.0f;
            velocities[3*offset+2] = 0.0f;
            particles[4*offset]      = pos.x();
            particles[4*offset+1]    = pos.y();
            particles[4*offset+2]    = pos.z();

        }
    }  
}


void createSpringsFromEdges(const GU_Detail &source, std::vector<int> &springIndices,
                      std::vector<float> &springLengths, std::vector<float> &springCoefficients,
                      const int springPerVertex=1)
{
    GA_ROHandleF     stiffness_handle(&source, GA_ATTRIB_POINT, "stiffness");
    for (GA_Iterator it(source.getPrimitiveRange()); !it.atEnd(); ++it)
    {
        const GEO_Primitive *prim = source.getGEOPrimitive(*it);
        const GA_Range vertices = prim->getVertexRange();
        GA_Offset vi;
        int springSpan = springPerVertex;
        if (vertices.getEntries() <= springSpan) {
            springSpan = vertices.getEntries() - springSpan-1; 
            springSpan = SYSmax(springSpan, 1);
        }
        for (vi = GA_Offset(0); vi < vertices.getEntries() - springSpan; ++vi) {           
            const GA_Offset vtx1 = prim->getVertexOffset(vi);
            for (int span=1; span<=springSpan; ++span) {
                const GA_Offset vtx2 = prim->getVertexOffset(vi+span);
                springIndices.push_back(static_cast<int>(vtx1));
                springIndices.push_back(static_cast<int>(vtx2));
                const UT_Vector3 vp1 = source.getPos3(vtx1);
                const UT_Vector3 vp2 = source.getPos3(vtx2);
                springLengths.push_back(UT_Vector3(vp2 - vp1).length());
                if (stiffness_handle.isValid()) {
                    const float stiffness1 = stiffness_handle.get(vtx1);
                    const float stiffness2 = stiffness_handle.get(vtx2);
                    springCoefficients.push_back((stiffness1+stiffness2)/2.0f);
                }
                else {
                    springCoefficients.push_back(.5f);
                }
            }
        }
    }
}


void createSpringsFromAttrib(const GU_Detail &source, std::vector<int> &springIndices,
                       std::vector<float> &springLengths,  std::vector<float> &springCoefficients)
{
    GA_ROHandleF  stiffness_handle(&source, GA_ATTRIB_POINT, "stiffness");
    GA_ROHandleIA springs_handle(&source, GA_ATTRIB_POINT, "springs");
   
    if (!springs_handle.isValid())
        return;

    GA_Offset ptoff;
    UT_IntArray springsOffsets;
    GA_FOR_ALL_PTOFF(&source, ptoff) {
        springs_handle.get(ptoff, springsOffsets);
        const UT_Vector3 vp1 = source.getPos3(ptoff);
        for (UT_IntArray::iterator it=springsOffsets.begin(); !it.atEnd(); ++it) {
            const UT_Vector3 vp2 = source.getPos3(*it);
            springIndices.push_back(ptoff);
            springIndices.push_back(*it);
            springLengths.push_back(UT_Vector3(vp2 - vp1).length());
            if (stiffness_handle.isValid()) {
                const float stiffness1 = stiffness_handle.get(ptoff);
                const float stiffness2 = stiffness_handle.get(*it);
                springCoefficients.push_back((stiffness1+stiffness2)/2.0f);
            }
            else {
                springCoefficients.push_back(.5f);
            } 
        }     
    }
}

void copySprings(const GU_Detail &gdp,  std::vector<int> &springIndices,
                       std::vector<float> &springLengths,  
                       std::vector<float> &springCoefficients)
{
    GA_ROHandleIA springs_handle(&gdp, GA_ATTRIB_POINT, "springs");
    if (springs_handle.isValid())
        createSpringsFromAttrib(gdp, springIndices, springLengths, springCoefficients);
    else
        createSpringsFromEdges(gdp, springIndices, springLengths, springCoefficients);
}


void copyMesh(const GU_Detail &gdp, float *points, int *vertices, \
            int &npoints, int &npoly, float *lower, float *upper)
{
    UT_BoundingBox bbox(0,0,0,1,1,1);
    if (gdp.getBBox(&bbox)) {
        lower[0] = bbox.xmin(); lower[1] = bbox.ymin(); 
        lower[2] = bbox.zmin(); upper[0] = bbox.xmax(); 
        upper[1] = bbox.ymax(); upper[2] = bbox.zmax();
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

void createCollisionMesh(FlexSolver *solver, const GU_Detail &collisionGeo, const int frame)
{
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
    float shapeAabbMaxs[4] = {upper[0], upper[1], upper[2], 0};
    float shapeAabbMins[4] = {lower[0], lower[1], lower[2], 0};
    int shapeOffsets[1]; shapeOffsets[0] = 0;
    float shapePositions[4] = {0,0,0,0};
    float shapeRotations[4] = {1,0,0,0};
    float shapePrevPositions[4] = {0,0,0,0};
    float shapePrevRotations[4] = {1,0,0,0};

    if (frame) {
        shapePositions[0]     = SYSsin(frame*2.0f)*.5;
        shapePrevPositions[0] = SYSsin((frame-1)*2.0f)*.5;
    }

    int shapeFlags[1]; 
    shapeFlags[0] = flexMakeShapeFlags(eFlexShapeTriangleMesh, true);
    int numShapes = 1;
    flexSetShapes(solver, &collisionGeometries[0], numGeometryEntries, shapeAabbMins, shapeAabbMaxs, 
        shapeOffsets, shapePositions, shapeRotations, shapePrevPositions, shapePrevRotations, \
            shapeFlags, numShapes, eFlexMemoryHostAsync);

    flexDestroyTriangleMesh(mesh);


    GU_Detail test;
    for (int i = 0; i < collisionGeo.getNumPoints(); ++i)
    {
        GA_Offset ptoff;
        ptoff = test.appendPointOffset();
        const UT_Vector3 pos = UT_Vector3(meshPoints[3*i], meshPoints[3*i+1], meshPoints[3*i+2]);
        test.setPos3(ptoff, pos);
    }
    test.save("./test.bgeo", 0, 0);
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

    const int maxParticles = gdp.getNumPoints();//+ collisionGeo.getNumPoints();
    const int maxDiffuse = 0;

    FlexSolver* solver = flexCreateSolver(maxParticles, maxDiffuse);
    FlexTimers timer   = FlexTimers();
    FlexParams parms   = FlexParams();
    // flexSetDebug(solver, true);
    // flexStartRecord(solver, "./debug");
    // parms.mPlanes[0][0] = 0.0f;
    // parms.mPlanes[0][1] = 1.0f;
    // parms.mPlanes[0][2] = 0.0f;
    // parms.mPlanes[0][3] = 0.0f;

    // parms.mNumPlanes = 1;

    // set Fluid like parms:
    InitFlexParams(parms);
    flexSetParams(solver, &parms);

    // alloc CUDA pinned host memory to allow asynchronous memory transfers

    std::vector<float> particles(maxParticles*4, 0);
    std::vector<float> velocities(maxParticles*4,0);
    std::vector<int>   actives(maxParticles, 0);
    std::vector<int>   phases(maxParticles, 0);

    for (int i=0; i < maxParticles; ++i) {
        actives[i] = i;
    }

    // set initial particle data
    copyPointAttribs(gdp, particles, velocities, actives, phases, -1.0);

    // Springs setup:
    // uint  numSprings     = getNumSprings(gdp);
    // std::cout << "Springs created      : " <<  numSprings <<std::endl;
    std::vector<int>   springIndices(0);        // = reinterpret_cast<int*>(flexAlloc(sizeof(int)*2*numSprings));
    std::vector<float> springLengths(0);        //= reinterpret_cast<float*>(flexAlloc(sizeof(float)*numSprings));
    std::vector<float> springCoefficients(0);   // =reinterpret_cast<float*>(flexAlloc(sizeof(float)*numSprings));
    copySprings(gdp, springIndices, springLengths, springCoefficients);


    flexSetActive(solver, &actives[0], maxParticles, eFlexMemoryHost);
    flexSetParticles(solver, &particles[0], maxParticles, eFlexMemoryHost);
    flexSetVelocities(solver, &velocities[0], maxParticles, eFlexMemoryHost);
    flexSetSprings(solver, &springIndices[0], &springLengths[0], \
        &springCoefficients[0], springLengths.size(), eFlexMemoryHost);


    /*------------- Collision objects ------------- */

    // createCollisionMesh(solver, collisionGeo, 0);
    
  
    GU_Detail source;
    source.copy(gdp);

    int counter = 1;
    while (counter <= 240)
    {
        const float dt = 1.0f/24.0f;

        // update positions, apply custom force fields, etc
        // ModifyParticles(particles, velocities);
        copyPointAttribs(source, particles, velocities, actives, phases, 1.0);
        // flexSetActive(solver, actives, maxParticles, eFlexMemoryHostAsync);

        // update GPU data asynchronously
        // const float t = counter / 24.0f;
        // createCollisionMesh(solver, collisionGeo, counter);
    
        flexSetParticles(solver, &particles[0], maxParticles, eFlexMemoryHost);
        flexSetVelocities(solver, &velocities[0], maxParticles, eFlexMemoryHost);
        flexSetSprings(solver, &springIndices[0], &springLengths[0], &springCoefficients[0], \
            springLengths.size(), eFlexMemoryHost);
        

        // tick solver
        flexUpdateSolver(solver, dt, 1, &timer);

        // kick off async memory reads from device
        flexGetParticles(solver, &particles[0], maxParticles, eFlexMemoryHost);
        flexGetVelocities(solver, &velocities[0], maxParticles, eFlexMemoryHost);

        // wait for GPU to finish working (can perform async. CPU work here)
        // flexSetFence();


        /*-----> Copy to/from Houdini here as GPU computes. <--------------*/


        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(&gdp, ptoff) {
            const int offset = static_cast<int>(ptoff)*4;
            // float *point     = &particles_ptr[offset];
            const UT_Vector3 pos(particles[offset], particles[offset+1], particles[offset+2]);
            gdp.setPos3(ptoff, pos);
        }
        // updated particle data is ready to be used
        saveGeometry(gdp, counter);

        // flexWaitFence();
        
        counter++;

            
    }

    std::cout << "Total time: " <<  timer.mTotal << std::endl;
    std::cout << "Density ca: " <<  timer.mCalculateDensity << std::endl;
    std::cout << "Collisions: " <<  timer.mCollideParticles << std::endl;

    // flexFree(particles);
    // flexFree(velocities);
    // flexFree(springIndices);
    // flexFree(springLengths);
    // flexFree(springCoefficients);
    // flexFree(actives);
    // flexFree(phases);
    // flexStopRecord(solver);
    flexDestroySolver(solver);
    flexShutdown();

    return 0;
}













