/*
 * Copyright (c) 2016
 *  Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 * This is a simple particle system...  Not too much to it, but this can be
 * used as a template for your own...
 */


#ifndef __SOP_SParticle_h__
#define __SOP_SParticle_h__

#include <SOP/SOP_Node.h>

#define INT_PARM(name, idx, vidx, t)    \
        return evalInt(name, &myOffsets[idx], vidx, t);

#define FLT_PARM(name, idx, vidx, t)    \
        return evalFloat(name, &myOffsets[idx], vidx, t);

class GEO_ParticleVertex;
class GEO_PrimParticle;
class GU_RayIntersect;

namespace HDK_Sample {
class SOP_SParticle : public SOP_Node
{
public:
    SOP_SParticle(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_SParticle();

    static PRM_Template      myTemplateList[];
    static OP_Node           *myConstructor(OP_Network*, const char *,
                                OP_Operator *);

protected:
    virtual const char          *inputLabel(unsigned idx) const;

    void        birthParticle();
    int         moveParticle(GA_Offset ptoff,
                     const UT_Vector3 &force);

    void        initSystem();
    void        timeStep(fpreal now);

    // Method to cook geometry for the SOP
    virtual OP_ERROR         cookMySop(OP_Context &context);

private:
    // These use defines to make it easy to add parms and remove them.
    // The evaluation routines use the indexed name lookup which is quite
    //  fast, yet easy to change indices (since the order of the indices
    //  doesn't have to be in sequential order...
    int          RESET()    { INT_PARM("reset", 0, 0, 0) }
    int          BIRTH(fpreal t){ INT_PARM("birth", 2, 0, t) }
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
    uint                 maxParticles;
    std::vector<float>   particles;
    std::vector<float>   velocities;
    std::vector<int>     actives;
    std::vector<int>     phases;
    static int          *myOffsets;
};


void InitFlexParams(FlexParams &g_params)
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
    g_params.mNumIterations = 2;
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


} // End HDK_Sample namespace

#endif
