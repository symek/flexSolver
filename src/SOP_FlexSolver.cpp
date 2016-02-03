


#include <flex.h>
// #include <flexExt.h>
#include <stddef.h>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <vector>
#include <math.h> // floor()
#include <time.h>
#include <stdlib.h> // for NULL wtf?"


// #include "cloth.h"
#include "SOP_FlexSolver.hpp"

#include <GU/GU_Detail.h>
#include <GU/GU_RayIntersect.h>

#include <GEO/GEO_PrimPart.h>

#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>

#include <PRM/PRM_Include.h>

#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_Vector4.h>

using namespace SOPFlexSolver;

void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "fleXSolver",
        "FleX Solver",
        SOP_FlexSolver::myConstructor,
        SOP_FlexSolver::myTemplateList,
        1,      // Min required sources
        2,      // Maximum sources
        0));
}

    // int   mNumIterations;                //!< Number of solver iterations to perform per-substep
    // float mGravity[3];                  //!< Constant acceleration applied to all particles

    // float mRadius;                      //!< The maximum interaction radius for particles
    // float mSolidRestDistance;           //!< The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius]
    // float mFluidRestDistance;           //!< The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius

// The names here have to match the inline evaluation functions
static PRM_Name names[] = {
    PRM_Name("reset",            "Reset Frame"),
    PRM_Name("maxParticles",     "Max Particles"),
    PRM_Name("numIterations",    "Solver interations"),
    PRM_Name("radius",           "Interact radius"),
    PRM_Name("solidRestDistance","Non-fluid particles radius"),
    PRM_Name("fluidRestDistance","Rest density"),



    PRM_Name("force",        "Force"),
};

static PRM_Default  MAXPARTICLESDEF(1024*1024);
static PRM_Default  RADIUSDEF(0.15);
static const char* resetFrameHelp        = "Set frame when simulation will reset.";
static const char* maxParticlesHelp      = "Maximum number of particles to be created on the GPU.";  
static const char* numIterationsHelp     = "Number of solver iterations to perform per-substep.";  
static const char* radiusHelp            = "The maximum interaction radius for particles.";  
static const char* solidRestDistanceHelp = "The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius].";  
static const char* fluidRestDistanceHelp = "The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius.";      


PRM_Template
SOP_FlexSolver::myTemplateList[] = {
    PRM_Template(PRM_INT,   1, &names[0], PRMoneDefaults, 0, 0, 0, 0, 1, resetFrameHelp),
    PRM_Template(PRM_INT_J, 1, &names[1], &MAXPARTICLESDEF, 0, 0, 0, 0, 1, maxParticlesHelp),
    PRM_Template(PRM_INT_J, 1, &names[2], PRMtwoDefaults, 0, 0, 0, 0, 1, numIterationsHelp),
    PRM_Template(PRM_FLT_J, 1, &names[3], &RADIUSDEF, 0, 0, 0, 0, 1, radiusHelp),
    PRM_Template(PRM_FLT_J, 1, &names[4], PRMzeroDefaults, 0, 0, 0, 0, 1, solidRestDistanceHelp),
    PRM_Template(PRM_FLT_J, 1, &names[5], PRMzeroDefaults, 0, 0, 0, 0, 1, fluidRestDistanceHelp),

    PRM_Template(PRM_XYZ_J, 3, &names[6]),
    PRM_Template(),
};

int *
SOP_FlexSolver::myOffsets = 0;

OP_Node *
SOP_FlexSolver::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_FlexSolver(net, name, op);
}

SOP_FlexSolver::SOP_FlexSolver(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
    , mySystem(NULL)
    , mySolver(NULL)
    , myTimer(NULL)
    , myParms(NULL)
    , maxParticles(1048576)
{
    // Make sure that our offsets are allocated.  Here we allow up to 32
    // parameters, no harm in over allocating.  The definition for this
    // function is in OP/OP_Parameters.h
    if (!myOffsets)
        myOffsets = allocIndirect(32);

    // Now, flag that nothing has been built yet...
    myVelocity.clear();

    FlexError status = flexInit();

    if(status)
    {
        switch(status)
        {
            case 0:
            break;
            case 1:
            std::cout << "FlexError: The header version does not match the library binary." << std::endl;
            return ;
            case 2:
            std::cout << "FlexError:The GPU associated with the calling thread does not meet requirements." << std::endl;
            std::cout << "An SM3.0 GPU or above is required" << std::endl;
            return ;
        }
    }
}

SOP_FlexSolver::~SOP_FlexSolver()
{
  
    // flexExtDestroyContainer(container);
    flexDestroySolver(mySolver);
    flexShutdown();
    if (myTimer)
        delete myTimer;
    if (myParms)
        delete myParms;
}


void
SOP_FlexSolver::timeStep(fpreal now)
{
    UT_Vector3 force(FX(now), FY(now), FZ(now));
    // int nbirth = BIRTH(now);

    if (error() >= UT_ERROR_ABORT)
        return;

   for (GA_Offset srcptoff = 0; srcptoff < maxParticles; ++srcptoff)
    {
        UT_Vector3 vel;
        if (mySourceVel.isValid())
            vel = mySourceVel.get(srcptoff);
        else
            vel = UT_Vector3(0,0,0);

        vel += force;
        uint index = static_cast<int>(srcptoff);
        uint v = index * 3;
        velocities[v]   = vel.x();
        velocities[v+1] = vel.y();
        velocities[v+2] = vel.z();
    }

    InitFlexParams(*myParms, now);
    flexSetParams(mySolver, myParms);
    flexSetVelocities(mySolver, &velocities[0], maxParticles, eFlexMemoryHost);

     const float dt = 1.0 / 24.0;
     const int substeps = 1;

    // tick solver
    flexUpdateSolver(mySolver, dt, substeps, myTimer);
    // update GPU data asynchronously
  
    flexGetParticles(mySolver, (float*)&particles[0], maxParticles, eFlexMemoryHost);

    for (GA_Offset srcptoff = 0; srcptoff < maxParticles; ++srcptoff)
    {
       uint p = static_cast<int>(srcptoff) * 4;
       const UT_Vector3 pos = UT_Vector3(particles[p], particles[p+1], particles[p+2]);
       gdp->setPos3(srcptoff, pos);
    }
}

void 
SOP_FlexSolver::resetGdp()
{
    // Check to see if we really need to reset everything
    if (gdp->getPointMap().indexSize() > 0 || myVelocity.isInvalid())
    {
        mySourceNum = 0;
        gdp->clearAndDestroy();
        mySystem = (GEO_PrimParticle *)gdp->appendPrimitive(GEO_PRIMPART);
        mySystem->clearAndDestroy();

        // A vector attribute will be transformed correctly by following
        //  SOPs.  Use float types for stuff like color...
        myVelocity = GA_RWHandleV3(gdp->addFloatTuple(GA_ATTRIB_POINT, "v", 3));
        if (myVelocity.isValid())
            myVelocity.getAttribute()->setTypeInfo(GA_TYPE_VECTOR);
        myLife = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "life", 2));
    }
}


void SOP_FlexSolver::copySourceParticles(bool copyUsed=false, bool copyGdp=true)
{
    if (mySource)
    {
        for (GA_Offset srcptoff = 0; srcptoff < maxParticles; ++srcptoff)
        {
            // GA_OffsetArray vertices;
            // NOTE: Copy only free points!
            if (!mySource->isPointUsed(srcptoff) || copyUsed)
            {
                const UT_Vector3 pos = mySource->getPos3(srcptoff);
                UT_Vector3 vel;
                if (mySourceVel.isValid())
                    vel = mySourceVel.get(srcptoff);
                else
                    vel = UT_Vector3(0,0,0);
                uint index = static_cast<int>(srcptoff);
                uint p = index * 4;
                uint v = index * 3;
                particles[p]    = pos.x();
                particles[p+1]  = pos.y();
                particles[p+2]  = pos.z();
                particles[p+3]  = 1.0;
                velocities[v]   = vel.x();
                velocities[v+1] = vel.y();
                velocities[v+2] = vel.z();

                if (copyGdp)
                {   
                    gdp->insertPointCopy(srcptoff);
                    gdp->setPos3(srcptoff, pos);
                }
            }
        }
    }

}

void SOP_FlexSolver::copyClothPrimitives()
{
    // copy points pos first, but dont copy them into gdp. 
    // copySourceParticles(true, false);
    // // gdp->duplicate(*mySource);   

    // GEO_Primitive *prim;
    // uint prims = mySource->getPrimitiveMap().indexSize();
    // uint points = mySource->getPointMap().indexSize();
    // indices.reserve(prims*3);

    // // GA_PrimitiveTypeMask mask = GA_PrimitiveTypeMask();
    // // mask.add(GA_PrimitiveTypeId(GEO_FAMILY_FACE));

    // GA_FOR_ALL_PRIMITIVES(mySource, prim)
    // {
    //     GA_Primitive::const_iterator vt;
    //     for (prim->beginVertex(vt); !vt.atEnd(); ++vt)
    //     {
    //         const GA_Offset srcptoff = vt.getPointOffset();
    //         indices.push_back(exint(srcptoff)); 
    //     }
    // }

    // FlexExtAsset* cloth;
    // float stretchStiffness =0.5f;
    // float bendStiffness = 0.5f;
    // float tetherStiffness = 0.0f;
    // float tetherGive = 0.0f;
    // float pressure = 0.0f;

    // cloth = flexExtCreateClothFromMesh(&particles[0], points, &indices[0], prims, 
    //     stretchStiffness, bendStiffness, tetherStiffness, tetherGive,  pressure);

    // int *i;
    // flexExtAllocParticles(container, points, i);
    // FlexExtInstance* instanceCloth;
    // const float transform[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    // int phase = flexMakePhase(0, eFlexPhaseSelfCollide);
    // int invMassScale = 1;
    // instanceCloth = flexExtCreateInstance(container, cloth, transform, 0, 0, 0, phase, invMassScale);
    





}

void
SOP_FlexSolver::initSystem(fpreal current_time)
{
    // Nothing to do here;
    if (!mySource)
        return;

    // Nothing to do here either;
    if (mySource->getPointMap().indexSize() == 0)
        return;

    // Reinit solver:
    if (mySolver)
    {
        flexDestroySolver(mySolver);
        if (myTimer) delete myTimer;
        if (myParms) delete myParms;
        mySolver = NULL;
        myTimer  = NULL;
        myParms  = NULL;
    }

    // No gdp
    if (!gdp) 
        gdp = new GU_Detail;

    // Create new solver:
    if (!mySolver)
    {
        mySolver  = flexCreateSolver(maxParticles,0);
        container = flexExtCreateContainer(mySolver, maxParticles);
        myTimer   = new FlexTimers();
        myParms   = new FlexParams();
    }

    // Clean previous geometry;
    resetGdp();

    // TMP: add collision ground:
    myParms->mPlanes[0][0] = 0.0f;
    myParms->mPlanes[0][1] = 1.0f;
    myParms->mPlanes[0][2] = 0.0f;
    myParms->mPlanes[0][3] = 0.0f;
    myParms->mNumPlanes = 1;

    // Set parameters for solver:
    InitFlexParams(*myParms, current_time);
    flexSetParams(mySolver, myParms);

    // Make sure we can handle particles count.
    uint nSourcePoints = mySource->getPointMap().indexSize();
    maxParticles       = SYSmin(maxParticles, nSourcePoints);

    // alloc CUDA pinned host memory to allow asynchronous memory transfers
    // Above isn't true TODO
    particles.resize(maxParticles*4, 0.0f);
    velocities.resize(maxParticles*3, 0.0f);
    actives.resize(maxParticles, 0.0f);

    int fluidPhase  = flexMakePhase(0, eFlexPhaseSelfCollide | eFlexPhaseFluid);
    phases.resize(maxParticles);

    // Create active particels set:
    for (int i=0; i < maxParticles; ++i)
    {
        actives[i] = i;
        phases[i] = fluidPhase;
    }


    // Copy source particles into self:
    copySourceParticles();

    // createMesh (cloth?):
    // copyClothPrimitives();
  
    // Initialize solver with sources:
    flexSetParticles(mySolver, &particles[0], maxParticles, eFlexMemoryHost);
    flexSetVelocities(mySolver, &velocities[0], maxParticles, eFlexMemoryHost);
    flexSetPhases(mySolver, &phases[0], maxParticles, eFlexMemoryHost);
    flexSetActive(mySolver, &actives[0], maxParticles, eFlexMemoryHost);
    

}

OP_ERROR
SOP_FlexSolver::cookMySop(OP_Context &context)
{
    // We must lock our inputs before we try to access their geometry.
    // OP_AutoLockInputs will automatically unlock our inputs when we return.
    // NOTE: Don't call unlockInputs yourself when using this!
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    // Now, indicate that we are time dependent (i.e. have to cook every
    // time the current frame changes).
    OP_Node::flags().timeDep = 1;

    // Channel manager has time info for us
    CH_Manager *chman = OPgetDirector()->getChannelManager();

    // This is the frame that we're cooking at...
    fpreal current_time = context.getTime();
    fpreal currframe = chman->getSample(context.getTime());
    fpreal reset = RESET(); // Find our reset frame...

    // Set up our source information...
    mySource = inputGeo(0, context);
    if (mySource)
    {
        mySourceVel = GA_ROHandleV3(mySource->findFloatTuple(GA_ATTRIB_POINT, "v", 3));

        // If there's no velocity, pick up the velocity from the normal
        if (mySourceVel.isInvalid())
        mySourceVel = GA_ROHandleV3(mySource->findFloatTuple(GA_ATTRIB_POINT, "N", 3));
    }

    if (currframe <= reset || !mySolver)
    {
        myLastCookTime = reset;
        initSystem(current_time);
    }
    else
    {
        //     // Set up the collision detection object
        //     const GU_Detail *collision = inputGeo(1, context);
        // if (collision)
        // {
        //     myCollision = new GU_RayIntersect;
        //     myCollision->init(collision);
        // }
        // else myCollision = 0;


        // This is where we notify our handles (if any) if the inputs have
        // changed.  This is normally done in cookInputGroups, but since there
        // is no input group, NULL is passed as the fourth argument.
        notifyGroupParmListeners(0, -1, mySource, NULL);

        // Now cook the geometry up to our current time
        // Here, we could actually re-cook the source input to get a moving
        // source...  But this is just an example ;-)

        currframe += 0.05;  // Add a bit to avoid floating point error
        while (myLastCookTime < currframe)
        {
            // Here we have to convert our frame number to the actual time.
            timeStep(chman->getTime(myLastCookTime));
            myLastCookTime += 1;
        }

        // if (myCollision) delete myCollision;

        // Set the node selection for the generated particles. This will 
        // highlight all the points generated by this node, but only if the 
        // highlight flag is on and the node is selected.
        select(GA_GROUP_POINT);
    }

    return error();
}

const char *
SOP_FlexSolver::inputLabel(unsigned inum) const
{
    switch (inum)
    {
    case 0: return "Particle Source Geometry.";
    case 1: return "Collision Objects.";
    }
    return "Unknown source";
}



// void
// SOP_SParticle::birthParticle()
// {
//     // Strictly speaking, we should be using mySource->getPointMap() for the
//     // initial invalid point, but mySource may be NULL.
//     GA_Offset srcptoff = GA_INVALID_OFFSET;
//     GA_Offset vtxoff = mySystem->giveBirth();
//     if (mySource)
//     {
//     if (mySourceNum >= mySource->getPointMap().indexSize())
//         mySourceNum = 0;
//     if (mySource->getPointMap().indexSize() > 0) // No points in the input
//         srcptoff = mySource->pointOffset(mySourceNum);
//     mySourceNum++; // Move on to the next source point...
//     }
//     GA_Offset ptoff = gdp->vertexPoint(vtxoff);
//     if (GAisValid(srcptoff))
//     {
//     gdp->setPos3(ptoff, mySource->getPos3(srcptoff));
//     if (mySourceVel.isValid())
//             myVelocity.set(ptoff, mySourceVel.get(srcptoff));
//     else
//             myVelocity.set(ptoff, UT_Vector3(0, 0, 0));
//     }
//     else
//     {
//         gdp->setPos3(ptoff, SYSdrand48()-.5, SYSdrand48()-.5, SYSdrand48()-.5);
//     myVelocity.set(ptoff, UT_Vector3(0, 0, 0));
//     }
//     // First index of the life variable represents how long the particle has
//     // been alive (set to 0).
//     myLife.set(ptoff, 0, 0);
//     // The second index of the life variable represents how long the particle
//     // will live (in frames)
//     myLife.set(ptoff, 1, 30+30*SYSdrand48());
// }

// int
// SOP_SParticle::moveParticle(GA_Offset ptoff, const UT_Vector3 &force)
// {
//     float life = myLife.get(ptoff, 0);
//     float death = myLife.get(ptoff, 1);
//     life += 1;
//     myLife.set(ptoff, life, 0); // Store back in point
//     if (life >= death)
//         return 0;               // The particle should die!

//     float tinc = 1./30.;        // Hardwire 1/30 of a second time inc...

//     // Adjust the velocity (based on the force) - of course, the multiplies
//     // can be pulled out of the loop...
//     UT_Vector3 vel = myVelocity.get(ptoff);
//     vel += tinc*force;
//     myVelocity.set(ptoff, vel);

//     // Now adjust the point positions

//     if (myCollision)
//     {
//     UT_Vector3 dir = vel * tinc;

//     // here, we only allow hits within the length of the velocity vector
//     GU_RayInfo info(dir.normalize());

//     UT_Vector3 start = gdp->getPos3(ptoff);
//     if (myCollision->sendRay(start, dir, info) > 0)
//         return 0;   // We hit something, so kill the particle
//     }

//     UT_Vector3 pos = gdp->getPos3(ptoff);
//     pos += tinc*vel;
//     gdp->setPos3(ptoff, pos);

//     return 1;
// }
