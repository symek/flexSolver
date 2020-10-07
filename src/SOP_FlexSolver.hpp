
#ifndef __SOP_FlexSolver_h__
#define __SOP_FlexSolver_h__

#include <SOP/SOP_Node.h>

#define INT_PARM(name, idx, vidx, t)    \
        return evalInt(name, &myOffsets[idx], vidx, t);

#define FLT_PARM(name, idx, vidx, t)    \
        return evalFloat(name, &myOffsets[idx], vidx, t);

class GEO_ParticleVertex;
class GEO_PrimParticle;
class GU_RayIntersect;

namespace SOPFlexSolver {

    using float4 = float[4];
    using float3 = float[3];

class SOP_FlexSolver : public SOP_Node
{
public:
    SOP_FlexSolver(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_FlexSolver();

    static PRM_Template      myTemplateList[];
    static OP_Node           *myConstructor(OP_Network*, const char *,
                                OP_Operator *);

protected:
    virtual const char          *inputLabel(unsigned idx) const;

    void        birthParticle();
    int         moveParticle(GA_Offset ptoff,
                     const UT_Vector3 &force);

    void        resetGdp();
    void        copySourceParticles();
    void        initSystem(fpreal);
    void        timeStep(fpreal now);

    // Method to cook geometry for the SOP
    virtual OP_ERROR         cookMySop(OP_Context &context);

private:
    // These use defines to make it easy to add parms and remove them.
    // The evaluation routines use the indexed name lookup which is quite
    //  fast, yet easy to change indices (since the order of the indices
    //  doesn't have to be in sequential order...
    int          RESET()               { INT_PARM("reset", 0, 0, 0) }
    int          MAXPARTICLES()        { INT_PARM("maxParticles", 2, 0, 0) }
    fpreal       RADIUS(fpreal t)              { FLT_PARM("radius", 1, 0, t) }
    fpreal       SOLIDRESTDISTANCE(fpreal t)   { FLT_PARM("solidRestDistance", 1, 0, t) }
    fpreal       FLUIDRESTDISTANCE(fpreal t)   { FLT_PARM("fluidRestDistance", 1, 0, t) }
   
    fpreal       FX(fpreal t)   { FLT_PARM("force", 1, 0, t) }
    fpreal       FY(fpreal t)   { FLT_PARM("force", 1, 1, t) }
    fpreal       FZ(fpreal t)   { FLT_PARM("force", 1, 2, t) }

    const GU_Detail *mySource;
    GA_Index         mySourceNum;       // Source point to birth from
    GA_ROHandleV3    mySourceVel;       // Velocity attrib in source

    GU_RayIntersect *myCollision;

    GEO_PrimParticle    *mySystem;
    fpreal               myLastCookTime;         // Last cooked time
    GA_RWHandleV3        myVelocity;             // My velocity attribute
    GA_RWHandleF         myLife;                 // My life attribute
    NvFlexLibrary*         library = nullptr;
    NvFlexSolver*          solver = nullptr;
    NvFlexTimers*          myTimer  = nullptr;
    NvFlexParams*          myParms  = nullptr;
    uint                 maxParticles;
    NvFlexBuffer* particlesBuffer;
    NvFlexBuffer* velocitiesBuffer;
    NvFlexBuffer* activesBuffer;
    NvFlexBuffer* phasesBuffer;
    static int   *myOffsets;

    void InitFlexParams(NvFlexParams &g_params, fpreal t)
    {
        g_params.gravity[0] = 0.0f;
        g_params.gravity[1] = -9.8f;
        g_params.gravity[2] = 0.0f;

        g_params.wind[0] = 0.0f;
        g_params.wind[1] = 0.0f;
        g_params.wind[2] = 0.0f;

        g_params.radius = RADIUS(t);
        g_params.viscosity = 0.0f;
        g_params.dynamicFriction = 0.0f;
        g_params.staticFriction = 0.0f;
        g_params.particleFriction = 0.0f; // scale friction between particles by default
        g_params.freeSurfaceDrag = 0.0f;
        g_params.drag = 0.0f;
        g_params.lift = 0.0f;
        g_params.numIterations = 2;
        g_params.fluidRestDistance = FLUIDRESTDISTANCE(t);
        g_params.solidRestDistance = SOLIDRESTDISTANCE(t);
        g_params.anisotropyScale = 1.0f;
        g_params.dissipation = 0.0f;
        g_params.damping = 0.0f;
        g_params.particleCollisionMargin = 0.0f;
        g_params.shapeCollisionMargin = 0.0f;
        g_params.collisionDistance = 0.0f;
//        g_params.plasticThreshold = 0.0f;
//        g_params.plasticCreep = 0.0f;
//        g_params.fluid = false;
        g_params.sleepThreshold = 0.0f;
        g_params.shockPropagation = 0.0f;
        g_params.restitution = 0.0f;
        g_params.smoothing = 1.0f;
        g_params.maxSpeed = 1000.0;
        g_params.relaxationMode = NvFlexRelaxationMode::eNvFlexRelaxationLocal;
        g_params.relaxationFactor = 1.0f;
        g_params.solidPressure = 1.0f;
        g_params.adhesion = 0.0f;
        g_params.cohesion = 0.025f;
        g_params.surfaceTension = 0.0f;
        g_params.vorticityConfinement = 0.0f;
        g_params.buoyancy = 1.0f;
        g_params.diffuseThreshold = 100.0f;
        g_params.diffuseBuoyancy = 1.0f;
        g_params.diffuseDrag = 0.8f;
        g_params.diffuseBallistic = 16;
//        g_params.diffuseSortAxis[0] = 0.0f;
//        g_params.diffuseSortAxis[1] = 0.0f;
//        g_params.diffuseSortAxis[2] = 0.0f;
//        g_params.enableCCD = false;

        g_params.dynamicFriction = 0.1f;
//        g_params.fluid = true;
        g_params.viscosity = 0.0f;
        g_params.numIterations = 2;
        g_params.vorticityConfinement = 0.0f;
        g_params.anisotropyScale = 25.0f;
        g_params.fluidRestDistance = g_params.radius*0.55f;


        // by default solid particles use the maximum radius
//        if (g_params.mFluid && g_params.solidRestDistance == 0.0f)
//            g_params.solidRestDistance = g_params.fluidRestDistance;
//        else
            g_params.solidRestDistance = g_params.radius;

        // collision distance with shapes half the radius
        if (g_params.collisionDistance == 0.0f)
        {
            g_params.collisionDistance = g_params.radius*0.5f;

//            if (g_params.fluid)
//                g_params.collisionDistance = g_params.fluidRestDistance*0.5f;
        }

        // default particle friction to 10% of shape friction
        if (g_params.particleFriction == 0.0f)
            g_params.particleFriction = g_params.dynamicFriction*0.1f;

        // add a margin for detecting contacts between particles and shapes
        if (g_params.shapeCollisionMargin == 0.0f)
            g_params.shapeCollisionMargin = g_params.collisionDistance*0.25f;
    }

};

} // End FlexSolver

#endif
