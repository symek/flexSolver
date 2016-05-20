


#include <flex.h>
#include <stddef.h>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <vector>
#include <math.h> // floor()
#include <time.h>
#include <stdlib.h> // for NULL wtf?"


#include "SOP_FlexWires.hpp"

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

using namespace SOPFlexWires;

void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "flexWires",
        "Flex Wires",
        SOP_FlexWires::myConstructor,
        SOP_FlexWires::myTemplateList,
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
    PRM_Name("resetframe",       "Reset Frame"),
    PRM_Name("maxParticles",     "Max Particles"),
    PRM_Name("numIterations",    "Solver iterations"),
    PRM_Name("radius",           "Interact radius"),
    PRM_Name("solidRestDistance","Non-fluid particles radius"),
    PRM_Name("maxspeed",          "Max speed."),
    //PRM_Name("fluidRestDistance","Rest density"),
    PRM_Name("force",        "Force"),
};

static PRM_Default  MAXPARTICLESDEF(1024*1024);
static PRM_Default  RADIUSDEF(0.05);
static PRM_Default  MAXSPEEDEF(1000);
static PRM_Default  RESTDISTANCE(0.1);

static const char* resetFrameHelp        = "Set frame when simulation will reset.";
static const char* maxParticlesHelp      = "Maximum number of particles to be created on the GPU.";  
static const char* numIterationsHelp     = "Number of solver iterations to perform per-substep.";  
static const char* radiusHelp            = "The maximum interaction radius for particles.";  
static const char* solidRestDistanceHelp = "The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius].";  
static const char* maxSpeedHelp          = "The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius.";      


PRM_Template
SOP_FlexWires::myTemplateList[] = {
    PRM_Template(PRM_INT,   1, &names[0], PRMoneDefaults, 0, 0, 0, 0, 1, resetFrameHelp),
    PRM_Template(PRM_INT_J, 1, &names[1], &MAXPARTICLESDEF, 0, 0, 0, 0, 1, maxParticlesHelp),
    PRM_Template(PRM_INT_J, 1, &names[2], PRMtwoDefaults, 0, 0, 0, 0, 1, numIterationsHelp),
    PRM_Template(PRM_FLT_J, 1, &names[3], &RADIUSDEF, 0, 0, 0, 0, 1, radiusHelp),
    PRM_Template(PRM_FLT_J, 1, &names[4], &RESTDISTANCE, 0, 0, 0, 0, 1, solidRestDistanceHelp),
    PRM_Template(PRM_FLT_J, 1, &names[5], &MAXSPEEDEF, 0, 0, 0, 0, 1, maxSpeedHelp),

    PRM_Template(PRM_XYZ_J, 3, &names[6]),
    PRM_Template(),
};

int *
SOP_FlexWires::myOffsets = 0;

OP_Node *
SOP_FlexWires::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_FlexWires(net, name, op);
}

SOP_FlexWires::SOP_FlexWires(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
    , mySystem(NULL)
    , mySolver(NULL)
    , myTimer(NULL)
    , myParms(NULL)
    , myMaxParticles(1048576) // Reasonable.
{
    // Make sure that our offsets are allocated.  Here we allow up to 32
    // parameters, no harm in over allocating.  The definition for this
    // function is in OP/OP_Parameters.h
    if (!myOffsets)
        myOffsets = allocIndirect(32);

    myFlexError = flexInit();
    interpretError(myFlexError);

    // Now, flag that nothing has been built yet...
    // myVelocity.clear();

}

SOP_FlexWires::~SOP_FlexWires()
{
    DEBUG_PRINT("%s\n", "Before freeing cuda data.");  
    #ifdef CUDA_ALLOCATOR 
    flexFree(&myParticles[0]);
    flexFree(&myVelocities[0]);
    flexFree(&myActives[0]);
    flexFree(&mySpringIndices[0]);
    flexFree(&mySpringLengths[0]);
    flexFree(&mySpringCoefficients[0]);
    #endif
    flexDestroySolver(mySolver);
    flexShutdown();
    DEBUG_PRINT("%s\n", "After shuting down Flex.");  
    if (myTimer)
        delete myTimer;
    if (myParms)
        delete myParms;
}

OP_ERROR
SOP_FlexWires::initSystem(fpreal currentTime)
{
    if (myFlexError) {
        addError(SOP_MESSAGE, "Flex can't be initilized.");
        return error();
    }

    // Make sure we can handle particles count.
    int nSourcePoints  = mySource->getNumPoints();
    int maxParticles   = nSourcePoints; //SYSmin(MAXPARTICLES(), nSourcePoints);
    myMaxParticles     = maxParticles; // FIXME: clean it.
    DEBUG_PRINT("%s: %i\n", "myMaxParticles", maxParticles);    

    if (mySolver) {
        flexDestroySolver(mySolver);
    }
        
    if (myTimer) {
            delete myTimer;
    }

    if (myParms) {
        delete myParms;
    }

    mySolver = NULL;
    myTimer  = NULL;
    myParms  = NULL;

    mySolver  = flexCreateSolver(myMaxParticles, 0);
    myTimer   = new FlexTimers();
    myParms   = new FlexParams();
    myParticles.resize(0);
    myVelocities.resize(0);
    myActives.resize(0);
    mySpringIndices.resize(0);
    mySpringLengths.resize(0);
    mySpringCoefficients.resize(0);

    DEBUG_PRINT("%s\n", "After reseting containers and solver.");
  
    // Set parameters for solver:
    DEBUG_PRINT("%s\n", "Before initFlexParms." );
    initFlexParms(*myParms, currentTime);
    flexSetParams(mySolver, myParms);


    DEBUG_PRINT("%s\n", "Before resizing containers." );
    // alloc CUDA pinned host memory to allow asynchronous memory transfers
    myParticles.resize(maxParticles*4, 0.0f);
    myVelocities.resize(maxParticles*3, 0.0f);
    myActives.resize(maxParticles, -1);

    // Actives
    for (int i=0; i<maxParticles; ++i) {
        myActives[i] = i;
    }

    uint numSprings = getNumSprings(mySource);
    DEBUG_PRINT("%s: %i\n", "NumSprings", numSprings);       
    mySpringIndices.resize(numSprings*2, 0);
    mySpringLengths.resize(numSprings, 0.0f);
    mySpringCoefficients.resize(numSprings, 0.0f);

    // Copy source particles into self:
    DEBUG_PRINT("%s\n", "Before coping points and springs attribs." );
    copyPointAttribs(gdp, myParticles, myVelocities, myActives, -1.0);
    copySpringAttribs(gdp, mySpringIndices, mySpringLengths, mySpringCoefficients);
  
    // Initialize solver with sources:
    DEBUG_PRINT("%s\n", "before flexSetParticles etc." );
    flexSetActive(mySolver, &myActives[0], maxParticles, eFlexMemoryHost);
    flexSetParticles(mySolver, &myParticles[0], maxParticles, eFlexMemoryHost);
    flexSetVelocities(mySolver, &myVelocities[0], maxParticles, eFlexMemoryHost);
    flexSetSprings(mySolver, &mySpringIndices[0], &mySpringLengths[0], &mySpringCoefficients[0], 
        numSprings, eFlexMemoryHost);
    

}

void
SOP_FlexWires::timeStep(fpreal now)
{
    // UT_Vector3 force(FX(now), FY(now), FZ(now));
//     // int nbirth = BIRTH(now);

    if (error() >= UT_ERROR_ABORT)
        return;

    DEBUG_PRINT("%s\n", "timeStep before initFlexParms." );
    // initFlexParms(*myParms, now);
    // flexSetParams(mySolver, myParms);
    // flexSetVelocities(mySolver, &velocities[0], maxParticles, eFlexMemoryHost);

     const float dt  = 1.0f/ OPgetDirector()->getChannelManager()->getSamplesPerSec();
     const int substeps = 1;

     DEBUG_PRINT("%s:, Now: %f, timestep: %f\n", "timeStep before copyPointAttribs.", now, dt );
     // update positions, apply custom force fields, etc
    updatePointAttribs(mySource, myParticles, myVelocities, myActives, 1.0);
    // flexSetActive(mySolver, &myActives[0], myMaxParticles, eFlexMemoryHostAsync);

    // update GPU data asynchronously
    // createCollisionMesh(solver, collisionGeo, counter);
    DEBUG_PRINT("%s\n", "timeStep before flexSetParticles." );
    flexSetParticles(mySolver,  &myParticles[0],  myMaxParticles, eFlexMemoryHost);
    flexSetVelocities(mySolver, &myVelocities[0], myMaxParticles, eFlexMemoryHost);
    flexSetSprings(mySolver, &mySpringIndices[0], &mySpringLengths[0], &mySpringCoefficients[0], 
        mySpringLengths.size(), eFlexMemoryHost);
        
    // tick solver
    DEBUG_PRINT("%s\n", "timeStep before flexUpdateSolver." );
    flexUpdateSolver(mySolver, dt, substeps, myTimer);
    // update GPU data asynchronously
  
    DEBUG_PRINT("%s\n", "timeStep before flexGetParticles." );
    flexGetParticles(mySolver, &myParticles[0], myMaxParticles, eFlexMemoryHost);
    flexGetVelocities(mySolver,&myVelocities[0],myMaxParticles, eFlexMemoryHost);

    DEBUG_PRINT("%s\n", "timeStep before flexSetFence." );
    // flexSetFence();

    DEBUG_PRINT("%s\n", "timeStep before copy data back to Houdini." );
    GA_RWHandleV3  vel_handle(gdp, GA_ATTRIB_POINT, "v");
    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
        const int offset = static_cast<int>(ptoff);
        const UT_Vector3 pos(myParticles[4*offset], 
                             myParticles[4*offset+1], 
                             myParticles[4*offset+2]);
        gdp->setPos3(ptoff, pos);
        if (vel_handle.isValid()) {
            const UT_Vector3 vel(myVelocities[3*offset],
                                 myVelocities[3*offset+1],
                                 myVelocities[3*offset+2]);
            vel_handle.set(ptoff, vel);

        }
    }
    DEBUG_PRINT("%s\n", "timeStep flexWaitFence." );
    // flexWaitFence();
}

void 
SOP_FlexWires::resetGdp(){}

void SOP_FlexWires::copySourceParticles(){}


OP_ERROR
SOP_FlexWires::cookMySop(OP_Context &context)
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
    fpreal currentTime  = context.getTime();
    fpreal currentFrame = chman->getSample(context.getTime());
    fpreal resetFrame   = RESETFRAME(); // Find our reset frame...
    //myMaxParticles      = MAXPARTICLES();

    // // Set up our source information...
   // return error();
    const bool sourceChanged = gdp->getNumPoints() != inputGeo(0)->getNumPoints() ;
    if ((currentFrame <= resetFrame) || !mySolver || sourceChanged)
    {
        mySource = inputGeo(0);
        // gdp->clearAndDestroy();
        duplicateSource(0, context);
        myLastCookTime = resetFrame;
        DEBUG_PRINT("%s\n", "Before initSystem.");
        initSystem(currentTime);
        DEBUG_PRINT("%s\n", "After initSystem." );

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
        // notifyGroupParmListeners(0, -1, mySource, NULL);

        // Now cook the geometry up to our current time
        // Here, we could actually re-cook the source input to get a moving
        // source...  But this is just an example ;-)

        // currentFrame += 0.05;  // Add a bit to avoid floating point error
        // while (myLastCookTime < currentFrame)
        // {
            // Here we have to convert our frame number to the actual time.
            DEBUG_PRINT("%s\n", "Before timeStep." );
            timeStep(chman->getTime(myLastCookTime));
            myLastCookTime += 1;
            DEBUG_PRINT("%s\n", "After timeStep." );
        // }

        // if (myCollision) delete myCollision;

        // Set the node selection for the generated particles. This will 
        // highlight all the points generated by this node, but only if the 
        // highlight flag is on and the node is selected.
        // select(GA_GROUP_POINT);
    }

    gdp->getP()->bumpDataId();

    return error();
}

const char *
SOP_FlexWires::inputLabel(unsigned inum) const
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
