
#ifndef __SOP_FlexWires_h__
#define __SOP_FlexWires_h__

#include <SOP/SOP_Node.h>
#include <UT/UT_DSOVersion.h>

#define INT_PARM(name, idx, vidx, t)    \
        return evalInt(name, &myOffsets[idx], vidx, t);

#define FLT_PARM(name, idx, vidx, t)    \
        return evalFloat(name, &myOffsets[idx], vidx, t);

#define MAX_PARAMETERS 32

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
    int          RESETFRAME(fpreal t)          { INT_PARM("resetframe",   0, 0, 0) }
    int          NUMITERATIONS(fpreal t)       { INT_PARM("numiterations",1, 0, t) }
    int          NUMSUBSTEPS(fpreal t)         { INT_PARM("numsubsteps",  2, 0, t) }
    int          MAXPARTICLES(fpreal t)        { INT_PARM("maxParticles", 3, 0, 0) }
    fpreal       RADIUS(fpreal t)              { FLT_PARM("radius",       4, 0, t) }
    fpreal       SOLIDRESTDISTANCE(fpreal t)   { FLT_PARM("solidRestDistance", 5, 0, t) }
    fpreal       DYNAMICFRICTION(fpreal t)     { FLT_PARM("dynamicfriction",   6, 0, t) }
    fpreal       STATICFRICTION(fpreal t)      { FLT_PARM("staticfriction",    7, 0, t) }
    fpreal       PARTICLEFRICTION(fpreal t)    { FLT_PARM("particlefriction",  8, 0, t) }
    fpreal       RESTITUTION(fpreal t)         { FLT_PARM("restitution",       9, 0, t) }
    fpreal       SLEEPTHRESHOLD(fpreal t)      { FLT_PARM("sleepthreshold",    10, 0, t) }
    fpreal       MAXSPEED(fpreal t)            { FLT_PARM("maxspeed",          11, 0, t) }

    fpreal       SHOCKPROPAGATION(fpreal t)    { FLT_PARM("shockpropagation",  12, 0, t) }
    fpreal       DISSIPATION(fpreal t)         { FLT_PARM("dissipation",       13, 0, t) }
    fpreal       DAMPING(fpreal t)             { FLT_PARM("damping",           14, 0, t) }
    fpreal       INERTIABIAS(fpreal t)         { FLT_PARM("inertiabias",       15, 0, t) }

    fpreal       COLLISIONDISTANCE(fpreal t)         { FLT_PARM("collisiondistance",       16, 0, t) }
    fpreal       PARTICLECOLLISIONMARGIN(fpreal t)   { FLT_PARM("particlecollisionmargin", 17, 0, t) }
    fpreal       SHAPECOLLISIONMARGIN(fpreal t)      { FLT_PARM("shapecollisionmargin",    18, 0, t) }
    int          MAXSPRINGWIRES(fpreal t)            { INT_PARM("maxspringwires",          19, 0, 0) }
    int          COLLISIONGROUND(fpreal t)           { INT_PARM("collisionground",         21, 0, 0) }


   
    fpreal       GX(fpreal t)   { FLT_PARM("gravity", 20, 0, t) }
    fpreal       GY(fpreal t)   { FLT_PARM("gravity", 20, 1, t) }
    fpreal       GZ(fpreal t)   { FLT_PARM("gravity", 20, 2, t) }

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
    int                  mySolverSubSteps;
    int                  myMaxSpringWires;
    int                  myCollisionGround;
    #ifdef CUDA_ALLOCATOR
    std::vector<float, FlexAllocator<float> >   myParticles;
    std::vector<float, FlexAllocator<float> >   myVelocities;
    std::vector<int,   FlexAllocator<int> >     myActives;
    std::vector<int,   FlexAllocator<int> >     myPhases;
    std::vector<int,   FlexAllocator<int> >     mySpringIndices;
    std::vector<float, FlexAllocator<float> >   mySpringLengths;
    std::vector<float, FlexAllocator<float> >   mySpringCoefficients;
    #else
    std::vector<float>   myParticles;
    std::vector<float>   myVelocities;
    std::vector<int>     myActives;
    std::vector<int>     myPhases;
    std::vector<int>     mySpringIndices;
    std::vector<float>   mySpringLengths;
    std::vector<float>   mySpringCoefficients;
    #endif
    static int          *myOffsets;

    void initFlexParms(FlexParams &g_params, fpreal t)
    {

        g_params.mNumIterations = NUMITERATIONS(t);
        g_params.mRelaxationMode = eFlexRelaxationLocal;
        g_params.mRelaxationFactor = 1.0;
        // max particles provided as func(maxpart)
        g_params.mGravity[0] = GX(t);
        g_params.mGravity[1] = GY(t);
        g_params.mGravity[2] = GZ(t);

        g_params.mRadius            = RADIUS(t);
        // g_params.mSolidRestDistance = SOLIDRESTDISTANCE(t);
        // g_params.mDynamicFriction   = DYNAMICFRICTION(t);
        // g_params.mStaticFriction    = STATICFRICTION(t);
        // g_params.mParticleFriction  = PARTICLEFRICTION(t);
        // g_params.mRestitution       = RESTITUTION(t);
        g_params.mSleepThreshold    = SLEEPTHRESHOLD(t);
        // g_params.mShockPropagation  = SHOCKPROPAGATION(t);
        // g_params.mDissipation       = DISSIPATION(t);
        g_params.mDamping           = DAMPING(t);
        // g_params.mInertiaBias       = INERTIABIAS(t);

        // g_params.mCollisionDistance       = COLLISIONDISTANCE(t);
        // g_params.mParticleCollisionMargin = PARTICLECOLLISIONMARGIN(t);
        // g_params.mShapeCollisionMargin    = SHAPECOLLISIONMARGIN(t);

        // cloth:
        g_params.mWind[0] = 0.0f;
        g_params.mWind[1] = 0.0f;
        g_params.mWind[2] = 0.0f;
        g_params.mDrag = 0.0f;
        g_params.mLift = 0.0f;


        g_params.mPlasticThreshold = 0.0f;
        g_params.mPlasticCreep = 0.0f;
        g_params.mFluid = false;
        // g_params.mViscosity = 0.0f;
        
        g_params.mSmoothing = 0.0f;
        g_params.mSolidPressure = 1.0f;
        g_params.mAdhesion = 0.0f;
        g_params.mCohesion = 0.025f;
        g_params.mSurfaceTension = 0.0f;
        g_params.mInertiaBias = 0.001f;

        // collision distance with shapes half the radius
        if (g_params.mCollisionDistance == 0.0f)
        {
            g_params.mCollisionDistance = g_params.mRadius*0.5f;
        }

        // default particle friction to 10% of shape friction
        if (g_params.mParticleFriction == 0.0f)
            g_params.mParticleFriction = g_params.mDynamicFriction*0.1f; 

        // add a margin for detecting contacts between particles and shapes
        if (g_params.mShapeCollisionMargin == 0.0f)
            g_params.mShapeCollisionMargin = g_params.mCollisionDistance*0.25f;

        g_params.mMaxSpeed = MAXSPEED(t);

        if (COLLISIONGROUND(t)) {

        g_params.mPlanes[0][0] = 0.0f;
        g_params.mPlanes[0][1] = 1.0f;
        g_params.mPlanes[0][2] = 0.0f;
        g_params.mPlanes[0][3] = 0.0f;
        g_params.mNumPlanes = 1;
        }
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


void copyPointAttribs(const GU_Detail &gdp, std::vector<float> &particles, 
                      std::vector<float> &velocities, std::vector<int> &actives, 
                      std::vector<int> &phases, const float glueToAnimation=0,
                      const FlexParams *parms=NULL)
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

void getPointNeighboursRecrusive(const GU_Detail &gdp, const GA_Offset ptoff, \
    std::set<GA_Offset> &pointList)
{
    GA_OffsetArray pointVertices;
    gdp.getVerticesReferencingPoint(pointVertices, ptoff);
    for(int i=0; i < pointVertices.size(); ++i) {
        const GA_Offset primoff   = gdp.vertexPrimitive(pointVertices(i));
        const GA_Primitive *prim  = gdp.getPrimitive(primoff);
        const GA_Range vertices   = prim->getPointRange();
        GA_Range::const_iterator it;
        for (it=vertices.begin(); !it.atEnd(); ++it) {
            GA_Offset v1, v2;
            if (prim->findEdgePoints(ptoff, *it, v1, v2))
                pointList.insert(*it);
        }
    }
}

void getPointNeighbours(const GU_Detail &gdp, const GA_Offset ptoff,\
                    std::set<GA_Offset> &pointList, const int steps=1) 
{
    getPointNeighboursRecrusive(gdp, ptoff, pointList);
    for (int i=1; i<steps; ++i) {
        std::set<GA_Offset> partial;
        std::set<GA_Offset>::const_iterator it;
        for(it=pointList.begin(); it!=pointList.end(); ++it) {
            getPointNeighboursRecrusive(gdp, *it, partial);
        }
        for(it=partial.begin(); it!=partial.end(); ++it)
            pointList.insert(*it);
    }
    pointList.erase(ptoff); //
}


void copySprings(const GU_Detail &source, std::vector<int> &springIndices,
        std::vector<float> &springLengths,  std::vector<float> &springCoefficients,
        const int edgeSpringSpan=1)
{
    GA_ROHandleF  stiffness_handle(&source, GA_ATTRIB_POINT, "stiffness");
    GA_ROHandleIA springs_handle(&source, GA_ATTRIB_POINT, "springs");
    UT_IntArray   springsOffsets;
    
    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(&source, ptoff) {
        std::set<GA_Offset> neighbours;
        if (springs_handle.isValid()) {
            springs_handle.get(ptoff, springsOffsets);
            for (UT_IntArray::iterator it=springsOffsets.begin(); !it.atEnd(); ++it)
                neighbours.insert(*it);
        }
        else {
            getPointNeighbours(source, ptoff, neighbours, edgeSpringSpan);
        }
        // FIXME: we will have repeated springs pairs atm...
        std::set<GA_Offset>::const_iterator it;
        for (it=neighbours.begin(); it!=neighbours.end(); ++it) {
            const UT_Vector3 vp1 = source.getPos3(ptoff);
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

/*TODO Remove. */
uint getNumSprings(const GU_Detail *gdp, const int springPerVertex=1)
{
    uint springs = 0;
    for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
        const GEO_Primitive *prim = gdp->getGEOPrimitive(*it);
        const GA_Range vertices = prim->getVertexRange();
        springs += (vertices.getEntries() - springPerVertex) * springPerVertex; 
    }
    return springs;
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






