#include <NvFlex.h>
#include <stddef.h>
#include <vector>

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
#include <UT/UT_Vector3.h>

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
    , mySystem(nullptr)
    , solver(nullptr)
    , myTimer(nullptr)
    , myParms(nullptr)
    , maxParticles(1048576) // Reasonable.
{
    // Make sure that our offsets are allocated.  Here we allow up to 32
    // parameters, no harm in over allocating.  The definition for this
    // function is in OP/OP_Parameters.h
    if (!myOffsets)
        myOffsets = allocIndirect(32);

    // Now, flag that nothing has been built yet...
    myVelocity.clear();
    NvFlexInitDesc desc;
    desc.deviceIndex = 0;
    desc.enableExtensions = 1;
    desc.renderDevice = 0;
    desc.renderContext = 0;
    desc.computeContext = 0;
    desc.computeType = eNvFlexCUDA;

    library = NvFlexInit(NV_FLEX_VERSION, ErrorCallback, &desc);
    if (NvFlexInit_failed || library == nullptr)
	{
		printf("Could not initialize Flex, exiting.\n");
		exit(-1);
	}

    NvFlexSolverDesc solverDesc;
    NvFlexSetSolverDescDefaults(&solverDesc);
    solverDesc.maxParticles = maxParticles;
    solverDesc.maxDiffuseParticles = 0;
    solver = NvFlexCreateSolver(library, &solverDesc);

    // TODO: add init error handling
}

SOP_FlexSolver::~SOP_FlexSolver() {
    NvFlexDestroySolver(solver);
    NvFlexShutdown(library);
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

    float3* velocities  = (float3*)NvFlexMap(velocitiesBuffer, eNvFlexMapWait);

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
        velocities[v][0]  = vel.x();
        velocities[v][1] = vel.y();
        velocities[v][2] = vel.z();
    }

    NvFlexUnmap(velocitiesBuffer);

    InitFlexParams(*myParms, now);
    NvFlexSetParams(solver, myParms);
    NvFlexSetVelocities(solver, velocitiesBuffer, nullptr);

     const float dt = 1.0 / 24.0;
     const int substeps = 1;

    // tick solver
    NvFlexUpdateSolver(solver, dt, substeps, myTimer);
    // update GPU data asynchronously
  
    NvFlexGetParticles(solver, particlesBuffer, nullptr);

    float3* particles  = (float3*)NvFlexMap(particlesBuffer, eNvFlexMapWait);
    for (GA_Offset srcptoff = 0; srcptoff < maxParticles; ++srcptoff)
    {
       uint p = static_cast<int>(srcptoff) * 4;
       const UT_Vector3 pos = UT_Vector3(particles[p][0], particles[p][1], particles[p][2]);
       gdp->setPos3(srcptoff, pos);
    }
    NvFlexUnmap(particlesBuffer);
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


void SOP_FlexSolver::copySourceParticles()
{
    float4* particles = (float4*)NvFlexMap(particlesBuffer, eNvFlexMapWait);
    float3* velocities = (float3*)NvFlexMap(velocitiesBuffer, eNvFlexMapWait);
//    int* phases = (int*)NvFlexMap(phasesBuffer, eFlexMapWait);

    if (mySource)
    {
        for (GA_Offset srcptoff = 0; srcptoff < maxParticles; ++srcptoff)
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
            particles[p][0]  = pos.x();
            particles[p][1]  = pos.y();
            particles[p][2]  = pos.z();
            particles[p][3]  = 1.0;
            velocities[v][0] = vel.x();
            velocities[v][1] = vel.y();
            velocities[v][2] = vel.z();

            gdp->insertPointCopy(srcptoff);
            gdp->setPos3(srcptoff, pos);
        }
    }
    // FIXME; should we unmap here?
}

void
SOP_FlexSolver::initSystem(fpreal current_time)
{
    if (!mySource)
        return;


    if (solver)
    {
        NvFlexDestroySolver(solver);
        if (myTimer)
            delete myTimer;
        if (myParms)
            delete myParms;
        solver = nullptr;
        myTimer  = nullptr;
        myParms  = nullptr;
    }


    if (!gdp) 
        gdp = new GU_Detail;

    if (!solver)
    {
        NvFlexSolverDesc solverDesc;
        NvFlexSetSolverDescDefaults(&solverDesc);
        solverDesc.maxParticles = maxParticles;
        solverDesc.maxDiffuseParticles = 0;
        solver = NvFlexCreateSolver(library, &solverDesc);
        myTimer   = new NvFlexTimers();
        myParms   = new NvFlexParams();
    }

    // Clean previous geometry;
    resetGdp();

    // TMP: add collision ground:
    myParms->planes[0][0] = 0.0f;
    myParms->planes[0][1] = 1.0f;
    myParms->planes[0][2] = 0.0f;
    myParms->planes[0][3] = 0.0f;
    myParms->numPlanes = 1;

    // Set parameters for solver:
    InitFlexParams(*myParms, current_time);
    NvFlexSetParams(solver, myParms);

    // Make sure we can handle particles count.
    uint nSourcePoints = mySource->getPointMap().indexSize();
    maxParticles       = SYSmin(maxParticles, nSourcePoints);

    // alloc CUDA pinned host memory to allow asynchronous memory transfers
    // Above isn't true TODO
    particlesBuffer  = NvFlexAllocBuffer(library, maxParticles, sizeof(float)*4, eNvFlexBufferHost);
    velocitiesBuffer = NvFlexAllocBuffer(library, maxParticles, sizeof(float)*4, eNvFlexBufferHost);
    //    actives.resize(maxParticles, 0.0f);

    int fluidPhase  = NvFlexMakePhase(0, NvFlexPhase::eNvFlexPhaseSelfCollide | NvFlexPhase::eNvFlexPhaseFluid);
    phasesBuffer  = NvFlexAllocBuffer(library, maxParticles, sizeof(int), eNvFlexBufferHost);
    int* phases = (int*)NvFlexMap(phasesBuffer, NvFlexMapFlags::eNvFlexMapWait);

    // Create active particels set:
    for (int i=0; i < maxParticles; ++i) {
        phases[i] = fluidPhase;
    }

    NvFlexUnmap(phasesBuffer);
    // Copy source particles into self:
    copySourceParticles();
    NvFlexUnmap(particlesBuffer);
    NvFlexUnmap(velocitiesBuffer);

    // Initialize solver with sources:
    NvFlexSetParticles(solver, particlesBuffer, nullptr );
    NvFlexSetVelocities(solver, velocitiesBuffer, nullptr);
    NvFlexSetPhases(solver, phasesBuffer, nullptr );
//    NvFlexSetActive(solver, reinterpret_cast<NvFlexBuffer *>(actives), nullptr );

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
    this->flags().setTimeDep(true);

    // Channel manager has time info for us
    CH_Manager *chman = OPgetDirector()->getChannelManager();

    // This is the frame that we're cooking at...
    const fpreal current_time = context.getTime();
          fpreal currframe = chman->getSample(current_time);
    const fpreal reset = RESET(); // Find our reset frame...
    maxParticles = MAXPARTICLES();

    // Set up our source information...
    mySource = inputGeo(0, context);
    if (mySource)
    {
        mySourceVel = GA_ROHandleV3(mySource->findFloatTuple(GA_ATTRIB_POINT, "v", 3));

        // If there's no velocity, pick up the velocity from the normal
        if (mySourceVel.isInvalid())
        mySourceVel = GA_ROHandleV3(mySource->findFloatTuple(GA_ATTRIB_POINT, "N", 3));
    }

    if (currframe <= reset || !solver)
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
