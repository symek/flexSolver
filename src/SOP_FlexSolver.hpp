
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
    void        copySourceParticles(bool, bool);
    void        copyClothPrimitives();
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
    FlexSolver*          mySolver; 
    FlexTimers*          myTimer; 
    FlexParams*          myParms;  
    FlexExtContainer*    container;
    std::vector<FlexExtInstance*> instances;
    uint                 maxParticles;
    std::vector<float>   particles;
    std::vector<float>   velocities;
    std::vector<int>     actives;
    std::vector<int>     phases;
    std::vector<int>     indices;
    static int          *myOffsets;

    void InitFlexParams(FlexParams &g_params, fpreal t)
    {
        g_params.mGravity[0] = 0.0f;
        g_params.mGravity[1] = -9.8f;
        g_params.mGravity[2] = 0.0f;

        g_params.mWind[0] = 0.0f;
        g_params.mWind[1] = 0.0f;
        g_params.mWind[2] = 0.0f;

        g_params.mRadius = RADIUS(t);
        g_params.mViscosity = 0.0f;
        g_params.mDynamicFriction = 0.0f;
        g_params.mStaticFriction = 0.0f;
        g_params.mParticleFriction = 0.0f; // scale friction between particles by default
        g_params.mFreeSurfaceDrag = 0.0f;
        g_params.mDrag = 0.0f;
        g_params.mLift = 0.0f;
        g_params.mNumIterations = 2;
        g_params.mFluidRestDistance = FLUIDRESTDISTANCE(t);
        g_params.mSolidRestDistance = SOLIDRESTDISTANCE(t);
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
        g_params.mMaxSpeed = 1000.0;
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

        g_params.mDynamicFriction = 0.1f;
        g_params.mFluid = true;
        g_params.mViscosity = 0.0f;     
        g_params.mNumIterations = 2;
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

};

} // End FlexSolver

#endif
