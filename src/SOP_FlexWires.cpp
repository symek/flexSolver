


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
    PRM_Name("resetframe", "Reset Frame"),
    PRM_Name("numiterations", "Num Iterations"),
    PRM_Name("maxParticles", "Max Particles"),
    PRM_Name("radius", "Radius"),
    PRM_Name("solidRestDistance", "Solid Restdistance"),
    PRM_Name("dynamicfriction", "Dynamic Friction"),
    PRM_Name("staticfriction", "Static Frition"),
    PRM_Name("particlefriction", "Particle Friction"),
    PRM_Name("restitution", "Restitution"),
    PRM_Name("sleepthreshold", "Sleep Threshold"),
    PRM_Name("maxspeed", "Max Speed"),
    PRM_Name("shockpropagation", "Shock Propagation"),
    PRM_Name("dissipation", "Dissipation"),
    PRM_Name("damping", "Damping"),
    PRM_Name("inertiabias", "Inertia Bias"),
    PRM_Name("collisiondistance", "Collision Distance"),
    PRM_Name("particlecollisionmargin", "Particle Collisionmargin"),
    PRM_Name("shapecollisionmargin", "Shape Collisionmargin"),
    PRM_Name("force",        "Force"),
};

static const char* resetFrameHelp        = "Set frame when simulation will reset.";
static const char* maxParticlesHelp      = "Maximum number of particles to be created on the GPU.";  
static const char* numIterationsHelp     = "Number of solver iterations to perform per-substep.";  
static const char* radiusHelp            = "The maximum interaction radius for particles.";  
static const char* solidRestDistanceHelp = "The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius].";  
static const char* maxSpeedHelp          = "The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius.";      

static PRM_Default  RESETFRAME_DEFAULT(1);            
static PRM_Default  NUMITERATIONS_DEFAULT(3);         
static PRM_Default  MAXPARTICLES_DEFAULT(1024*1024);          
static PRM_Default  RADIUS_DEFAULT(0.01);   

static PRM_Default  SOLIDRESTDISTANCE_DEFAULT(0.0f);
static PRM_Default  DYNAMICFRICTION_DEFAULT(0.0f);
static PRM_Default  STATICFRICTION_DEFAULT(0.0f);
static PRM_Default  PARTICLEFRICTION_DEFAULT(0.0f); 
static PRM_Default  RESTITUTION_DEFAULT(0.0f);  

static PRM_Default  SLEEPTHRESHOLD_DEFAULT(0.0f);
static PRM_Default  MAXSPEED_DEFAULT(1000); 

static PRM_Default  SHOCKPROPAGATION_DEFAULT(0.0f);
static PRM_Default  DISSIPATION_DEFAULT(0.0f);
static PRM_Default  DAMPING_DEFAULT(0.0f);
static PRM_Default  INERTIABIAS_DEFAULT(0.0f);

static PRM_Default  COLLISIONDISTANCE_DEFAULT(0.0f);
static PRM_Default  PARTICLECOLLISIONMARGIN_DEFAULT(0.0f);
static PRM_Default  SHAPECOLLISIONMARGIN_DEFAULT(0.0f);

PRM_Template
SOP_FlexWires::myTemplateList[] = {
    PRM_Template(PRM_INT,   1, &names[0], PRMoneDefaults, 0, 0, 0, 0, 1, resetFrameHelp),
    PRM_Template(PRM_INT,   1, &names[1], &NUMITERATIONS_DEFAULT, 0, 0, 0, 0, 1, numIterationsHelp),
    PRM_Template(PRM_INT_J, 1, &names[2], &MAXPARTICLES_DEFAULT, 0, 0, 0, 0, 1, maxParticlesHelp),
    PRM_Template(PRM_FLT_J, 1, &names[3], &RADIUS_DEFAULT, 0, 0, 0, 0, 1, radiusHelp),

    PRM_Template(PRM_FLT_J, 1, &names[4], &SOLIDRESTDISTANCE_DEFAULT, 0, 0, 0, 0, 1, solidRestDistanceHelp),
    PRM_Template(PRM_FLT_J, 1, &names[5], &DYNAMICFRICTION_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[6], &STATICFRICTION_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[7], &PARTICLEFRICTION_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[8], &RESTITUTION_DEFAULT, 0, 0, 0, 0, 1, 0),

    PRM_Template(PRM_FLT_J, 1, &names[9], &SLEEPTHRESHOLD_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[10],&MAXSPEED_DEFAULT, 0, 0, 0, 0, 1, maxSpeedHelp),

    PRM_Template(PRM_FLT_J, 1, &names[11], &SHOCKPROPAGATION_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[12], &DISSIPATION_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[13], &DAMPING_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[14], &INERTIABIAS_DEFAULT, 0, 0, 0, 0, 1, 0),

    PRM_Template(PRM_FLT_J, 1, &names[15], &COLLISIONDISTANCE_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[16], &PARTICLECOLLISIONMARGIN_DEFAULT, 0, 0, 0, 0, 1, 0),
    PRM_Template(PRM_FLT_J, 1, &names[17], &SHAPECOLLISIONMARGIN_DEFAULT, 0, 0, 0, 0, 1, 0),

    //PRM_Template(PRM_XYZ_J, 3, &names[6]),
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
    copyPointAttribs(gdp, &myParticles[0], &myVelocities[0], &myActives[0], -1.0);
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

    // Set parameters for solver:
    DEBUG_PRINT("%s\n", "Before initFlexParms inside timeStep." );
    initFlexParms(*myParms, now);
    flexSetParams(mySolver, myParms);

    DEBUG_PRINT("%s:, Now: %f, timestep: %f\n", "timeStep before copyPointAttribs.", now, dt );
    // update positions, apply custom force fields, etc
    copyPointAttribs(mySource, &myParticles[0], &myVelocities[0], &myActives[0], 1.0);
    // flexSetActive(mySolver, &myActives[0], myMaxParticles, eFlexMemoryHost);

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
    fpreal resetFrame   = RESETFRAME(currentFrame); // Find our reset frame...
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

