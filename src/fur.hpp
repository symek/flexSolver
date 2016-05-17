void InitFlexParams(FlexParams &g_params)
    {
        g_params.mGravity[0] = 0.0f;
        g_params.mGravity[1] = -9.8f;
        g_params.mGravity[2] = 0.0f;

        g_params.mWind[0] = 0.0f;
        g_params.mWind[1] = 0.0f;
        g_params.mWind[2] = 0.0f;

        g_params.mRadius = 1; //RADIUS(t);
        g_params.mViscosity = 0.0f;
        g_params.mDynamicFriction = 1.0f;
        g_params.mStaticFriction = 1.0f;
        g_params.mParticleFriction = 1.0f; // scale friction between particles by default
        g_params.mFreeSurfaceDrag = 0.0f;
        g_params.mDrag = 1.0f;
        g_params.mLift = 0.0f;
        g_params.mNumIterations = 2;
        g_params.mFluidRestDistance = 0.1f; //FLUIDRESTDISTANCE(t);
        g_params.mSolidRestDistance = 0.1f; //SOLIDRESTDISTANCE(t);
        g_params.mAnisotropyScale = 1.0f;
        g_params.mDissipation = 0.0f;
        g_params.mDamping = 0.1f;
        g_params.mParticleCollisionMargin = 0.02f;
        g_params.mShapeCollisionMargin = 0.02f;
        g_params.mCollisionDistance = 0.2f;
        g_params.mPlasticThreshold = 0.0f;
        g_params.mPlasticCreep = 0.0f;
        g_params.mFluid = false;
        g_params.mSleepThreshold = 0.0f;
        g_params.mShockPropagation = 0.0f;
        g_params.mRestitution = 0.0f;
        g_params.mSmoothing = 0.0f;
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
        // g_params.mEnableCCD = false;

        g_params.mDynamicFriction = 0.1f;
        g_params.mFluid = false;
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
    };