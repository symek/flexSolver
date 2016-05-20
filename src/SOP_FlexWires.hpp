
#ifndef __SOP_FlexWires_h__
#define __SOP_FlexWires_h__

#include <SOP/SOP_Node.h>
#include <UT/UT_DSOVersion.h>

#define INT_PARM(name, idx, vidx, t)    \
        return evalInt(name, &myOffsets[idx], vidx, t);

#define FLT_PARM(name, idx, vidx, t)    \
        return evalFloat(name, &myOffsets[idx], vidx, t);

#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while (0)
#endif

// #define CUDA_ALLOCATOR 

class GEO_ParticleVertex;
class GEO_PrimParticle;
class GU_RayIntersect;
// class FlexAllocator;

template <class T> 
struct FlexAllocator {
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef T value_type;

    template <class U> struct rebind { typedef FlexAllocator<U> other; };
    FlexAllocator() throw() {}
    FlexAllocator(const FlexAllocator&) throw() {}

    template <class U> FlexAllocator(const FlexAllocator<U>&) throw(){}

    ~FlexAllocator() throw() {}

    pointer address(reference x) const { return &x; }
    const_pointer address(const_reference x) const { return &x; }

    pointer allocate(size_type s, void const * = 0) {
        if (0 == s)
            return NULL;
        pointer temp = (pointer)flexAlloc(s * sizeof(T)); 
        if (temp == NULL)
            throw std::bad_alloc();
        return temp;
    }

    void deallocate(pointer p, size_type) {
        /* this crashes perhaps due to solver being destroyed first?*/
        // flexFree(p);
    }

    size_type max_size() const throw() { 
        return std::numeric_limits<size_t>::max() / sizeof(T); 
    }

    void construct(pointer p, const T& val) {
        new((void *)p) T(val);
    }

    void destroy(pointer p) {
        p->~T();
    }
};

template <class T, class U>
bool operator==(const FlexAllocator<T>&, const FlexAllocator<U>&);
template <class T, class U>
bool operator!=(const FlexAllocator<T>&, const FlexAllocator<U>&);

namespace SOPFlexWires {

class SOP_FlexWires : public SOP_Node
{
public:
    SOP_FlexWires(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_FlexWires();

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
    OP_ERROR        initSystem(fpreal);
    void        timeStep(fpreal now);

    // Method to cook geometry for the SOP
    virtual OP_ERROR         cookMySop(OP_Context &context);

private:
    // These use defines to make it easy to add parms and remove them.
    // The evaluation routines use the indexed name lookup which is quite
    //  fast, yet easy to change indices (since the order of the indices
    //  doesn't have to be in sequential order...
    int          RESETFRAME()                  { INT_PARM("resetframe", 0, 0, 0) }
    int          MAXPARTICLES()                { INT_PARM("maxParticles", 2, 0, 0) }
    fpreal       RADIUS(fpreal t)              { FLT_PARM("radius", 1, 0, t) }
    fpreal       SOLIDRESTDISTANCE(fpreal t)   { FLT_PARM("solidRestDistance", 1, 0, t) }
    fpreal       MAXSPEED(fpreal t)            { FLT_PARM("maxspeed", 1, 0, t) }
   
    fpreal       FX(fpreal t)   { FLT_PARM("force", 1, 0, t) }
    fpreal       FY(fpreal t)   { FLT_PARM("force", 1, 1, t) }
    fpreal       FZ(fpreal t)   { FLT_PARM("force", 1, 2, t) }

    const GU_Detail *mySource;
    const GU_Detail *collider;
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
    FlexError            myFlexError;  
    int                  myMaxParticles;
    #ifdef CUDA_ALLOCATOR
    std::vector<float, FlexAllocator<float> >   myParticles;
    std::vector<float, FlexAllocator<float> >   myVelocities;
    std::vector<int,   FlexAllocator<int> >     myActives;
    std::vector<int,   FlexAllocator<int> >     mySpringIndices;
    std::vector<float, FlexAllocator<float> >   mySpringLengths;
    std::vector<float, FlexAllocator<float> >   mySpringCoefficients;
    #else
    std::vector<float>   myParticles;
    std::vector<float>   myVelocities;
    std::vector<int>     myActives;
    std::vector<int>     mySpringIndices;
    std::vector<float>   mySpringLengths;
    std::vector<float>   mySpringCoefficients;
    #endif
    static int          *myOffsets;

    void initFlexParms(FlexParams &g_params, fpreal t)
    {

         g_params.mGravity[0] = 0.0f;
        g_params.mGravity[1] = -9.8f;
        g_params.mGravity[2] = 0.0f;

        g_params.mWind[0] = 0.0f;
        g_params.mWind[1] = 0.0f;
        g_params.mWind[2] = 0.0f;

        g_params.mRadius = .01f; //RADIUS(t);
        g_params.mViscosity = 0.0f;
        g_params.mDynamicFriction = 0.0f;
        g_params.mStaticFriction = 0.0f;
        g_params.mParticleFriction = 0.0f; // scale friction between particles by default
        g_params.mFreeSurfaceDrag = 0.0f;
        g_params.mDrag = 0.0f;
        g_params.mLift = 0.0f;
        g_params.mNumIterations = 3;
        g_params.mFluidRestDistance = 0.1f; //FLUIDRESTDISTANCE(t);
        g_params.mSolidRestDistance = 0.1f; //SOLIDRESTDISTANCE(t);
        g_params.mAnisotropyScale = 1.0f;
        g_params.mDissipation = 0.0f;
        g_params.mDamping = 0.0f;
        g_params.mParticleCollisionMargin = 0.01f;
        g_params.mShapeCollisionMargin = 0.1f;
        g_params.mCollisionDistance = 0.01f;
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
        g_params.mInertiaBias = 0.001f;
        // g_params.mEnableCCD = false;

        g_params.mDynamicFriction = 0.1f;
        g_params.mFluid = false;
        g_params.mViscosity = 0.0f;     
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
    // }
    //     g_params.mGravity[0] = 0.0f;
    //     g_params.mGravity[1] = -9.8f;
    //     g_params.mGravity[2] = 0.0f;

    //     g_params.mWind[0] = 0.0f;
    //     g_params.mWind[1] = 0.0f;
    //     g_params.mWind[2] = 0.0f;

    //     g_params.mRadius = RADIUS(t);
    //     g_params.mViscosity = 0.0f;
    //     g_params.mDynamicFriction = 0.1f;
    //     g_params.mStaticFriction = 0.0f;
    //     g_params.mParticleFriction = 0.0f; // scale friction between particles by default
    //     g_params.mFreeSurfaceDrag = 0.0f;
    //     g_params.mDrag = 0.0f;
    //     g_params.mLift = 0.0f;
    //     g_params.mNumIterations = 1;
    //     // g_params.mFluidRestDistance = FLUIDRESTDISTANCE(t);
    //     g_params.mSolidRestDistance = SOLIDRESTDISTANCE(t);

    //     g_params.mAnisotropyScale = 1.0f;
    //     g_params.mAnisotropyMin = 0.1f;
    //     g_params.mAnisotropyMax = 2.0f;
    //     g_params.mSmoothing = 1.0f;

    //     g_params.mDissipation = 0.0f;
    //     g_params.mDamping = 0.0f;
    //     g_params.mParticleCollisionMargin = 0.0f;
    //     g_params.mShapeCollisionMargin = 0.0f;
    //     g_params.mCollisionDistance = 0.0f;
    //     g_params.mPlasticThreshold = 0.0f;
    //     g_params.mPlasticCreep = 0.0f;

    //     g_params.mFluid = false;

    //     g_params.mSleepThreshold = 0.0f;
    //     g_params.mShockPropagation = 0.0f;
    //     g_params.mRestitution = 0.0f;

    //     g_params.mMaxSpeed = MAXSPEED(t);
    //     g_params.mRelaxationMode = eFlexRelaxationLocal;
    //     g_params.mRelaxationFactor = 1.0f;

    //     g_params.mSolidPressure = 1.0f;
    //     g_params.mAdhesion = 0.0f;
    //     g_params.mCohesion = 0.025f;
    //     g_params.mSurfaceTension = 0.0f;
    //     g_params.mVorticityConfinement = 0.0f;
    //     g_params.mBuoyancy = 1.0f;
    //     g_params.mDiffuseThreshold = 100.0f;
    //     g_params.mDiffuseBuoyancy = 1.0f;
    //     g_params.mDiffuseDrag = 0.8f;
    //     g_params.mDiffuseBallistic = 16;
    //     g_params.mDiffuseSortAxis[0] = 0.0f;
    //     g_params.mDiffuseSortAxis[1] = 0.0f;
    //     g_params.mDiffuseSortAxis[2] = 0.0f;
    //     g_params.mDiffuseLifetime = 2.0f;
    //     g_params.mInertiaBias = 0.001f;

    //     g_params.mNumPlanes = 0;


    //     // by default solid particles use the maximum radius
    //     if (g_params.mFluid && g_params.mSolidRestDistance == 0.0f)
    //         g_params.mSolidRestDistance = g_params.mFluidRestDistance;
    //     else
    //         g_params.mSolidRestDistance = g_params.mRadius;

    //     // collision distance with shapes half the radius
    //     if (g_params.mCollisionDistance == 0.0f)
    //     {
    //         g_params.mCollisionDistance = g_params.mRadius*0.5f;

    //         if (g_params.mFluid)
    //             g_params.mCollisionDistance = g_params.mFluidRestDistance*0.5f;
    //     }

    //     // default particle friction to 10% of shape friction
    //     if (g_params.mParticleFriction == 0.0f)
    //         g_params.mParticleFriction = g_params.mDynamicFriction*0.1f; 

    //     // add a margin for detecting contacts between particles and shapes
    //     if (g_params.mShapeCollisionMargin == 0.0f)
    //         g_params.mShapeCollisionMargin = g_params.mCollisionDistance*0.25f;
    }

};

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

void copyPointAttribs(const GU_Detail *source, std::vector<float> &particles, 
                      std::vector<float>  &velocities, std::vector<int> &actives, 
                      const float pinToAnimation=0)
{
    GA_ROHandleV3  vel_handle(source, GA_ATTRIB_POINT, "v");
    GA_ROHandleF   pin_handle(source, GA_ATTRIB_POINT, "pintoanimation");
    GA_ROHandleF   mas_handle(source, GA_ATTRIB_POINT, "mass");
    const int      vel_valid = vel_handle.isValid();
          int      pin_valid = pin_handle.isValid();
    const int      mas_valid = mas_handle.isValid();

    float pinValue = 0.0f;
    float masValue = 1.0f;

    if (pinToAnimation < 0.0f) {
        pin_valid = 0;
        pinValue = 1.0f;
    }

    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(source, ptoff) 
    {
        const int offset = static_cast<const int>(ptoff);
        if (pin_valid) {
            pinValue = pin_handle.get(ptoff);
        if (pinValue == 1.0 && pinToAnimation > 0)
            actives[offset] = -1;
        }
        if (mas_valid) {
            masValue = mas_handle.get(ptoff);
        }

        const UT_Vector3 pos = source->getPos3(ptoff);

        UT_Vector3 delta(particles[4*offset],
                         particles[4*offset+1],
                         particles[4*offset+2]);
        // FIXME: Partial pinning doesn't work for now. 
        // delta -= (delta - pos)*pinValue;
        if (pinValue)
            delta = pos;

        particles[4*offset]   = delta.x();
        particles[4*offset+1] = delta.y();
        particles[4*offset+2] = delta.z();
        particles[4*offset+3] = masValue;
        
        if (vel_valid) {
            const UT_Vector3 vel    = vel_handle.get(ptoff);
            // NOTE: We'd better initialize velocities...
            velocities[3*offset]   += static_cast<float>(vel.x());//*pinValue; // No parial pinning 
            velocities[3*offset+1] += static_cast<float>(vel.y());//*pinValue;
            velocities[3*offset+2] += static_cast<float>(vel.z());//*pinValue;

        if (pinValue == 1.0f) {
            velocities[3*offset]   = 0.0f; 
            velocities[3*offset+1] = 0.0f;
            velocities[3*offset+2] = 0.0f;

            }
        }
    }  
}

void updatePointAttribs(const GU_Detail *source, std::vector<float> &particles, 
                      std::vector<float>  &velocities, std::vector<int> &actives, 
                      const float pinToAnimation=0)
{
    GA_ROHandleV3  vel_handle(source, GA_ATTRIB_POINT, "v");
    GA_ROHandleF   pin_handle(source, GA_ATTRIB_POINT, "pintoanimation");
    GA_ROHandleF   mas_handle(source, GA_ATTRIB_POINT, "mass");
    const int      vel_valid = vel_handle.isValid();
          int      pin_valid = pin_handle.isValid();
    const int      mas_valid = mas_handle.isValid();

    float pinValue = 1.0f;
    float masValue = 1.0f;
    UT_Vector3 vel(0, 0, 0);

    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(source, ptoff) 
    {
        const int offset = static_cast<const int>(ptoff);
        if (pin_valid) {
            pinValue = pin_handle.get(ptoff);
        if (pinValue == 1.0 && pinToAnimation > 0)
            actives[offset] = -1;
        }
        if (mas_valid) {
            masValue = mas_handle.get(ptoff);
        }
        if (vel_valid){
            vel = vel_handle.get(ptoff);
        }

        // const UT_Vector3 pos = source->getPos3(ptoff);

        particles[4*offset+3] = masValue;

        velocities[3*offset]   += static_cast<float>(vel.x());//*pinValue; // No parial pinning 
        velocities[3*offset+1] += static_cast<float>(vel.y());//*pinValue;
        velocities[3*offset+2] += static_cast<float>(vel.z());//*pinValue;

        if (pinValue == 1.0f) {
            velocities[3*offset]   = 0.0f; 
            velocities[3*offset+1] = 0.0f;
            velocities[3*offset+2] = 0.0f;

            }
    }  
}


void copySpringAttribs(const GU_Detail *source,  std::vector<int> &springIndices,
                      std::vector<float> &springLengths, std::vector<float> &springCoefficients)
{
    uint springCounter = 0;
    GA_ROHandleF     stiffness_handle(source, GA_ATTRIB_POINT, "stiffness");
    for (GA_Iterator it(source->getPrimitiveRange()); !it.atEnd(); ++it)
    {
        const GEO_Primitive *prim = source->getGEOPrimitive(*it);
        const GA_Range vertices = prim->getVertexRange();
        GA_Offset vi;
        for (vi = GA_Offset(1); vi < vertices.getEntries(); ++vi) {           
            const GA_Offset vtx1 = prim->getVertexOffset(vi-1);
            const GA_Offset vtx2 = prim->getVertexOffset(vi);
            springIndices[2*springCounter]     = static_cast<int>(vtx1);
            springIndices[(2*springCounter)+1] = static_cast<int>(vtx2);
            const UT_Vector3 vp1 = source->getPos3(vtx1);
            const UT_Vector3 vp2 = source->getPos3(vtx2);
            springLengths[springCounter] = UT_Vector3(vp2 - vp1).length();
            if (stiffness_handle.isValid()) {
                const float stiffness1 = stiffness_handle.get(vtx1);
                const float stiffness2 = stiffness_handle.get(vtx2);
                springCoefficients[springCounter] = (stiffness1+stiffness2)/2.0f;
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

uint getNumSprings(const GU_Detail *gdp)
{
    uint springs = 0;
    for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
        const GEO_Primitive *prim = gdp->getGEOPrimitive(*it);
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
}

} // End of SOPFlexWires
#endif






