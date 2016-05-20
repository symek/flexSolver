void InitFlexParams(FlexParams &g_params)
    {
        g_params.mGravity[0] = 0.0f;
        g_params.mGravity[1] = -9.8f;
        g_params.mGravity[2] = 0.0f;

        g_params.mWind[0] = 0.0f;
        g_params.mWind[1] = 0.0f;
        g_params.mWind[2] = 100.0f;

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
    };

    // template <typename T>
    // class CudaAllocator
    // {
    // public:
    //     void*
    //     allocate(uint32_t size)
    //     {
    //         assert(size);
    //         return flexAlloc(static_cast<size_t>(size));
    //     }
    //     void
    //     deallocate(void* ptr)
    //     {
    //         flexFree(ptr);
    //     }
    // };


// #include <stdlib.h>
// #include <new>
// #include <limits>


// #include <cstddef>
// template <class T>
// struct flexAllocator {
//   typedef T value_type;
//   flexAllocator(/*ctor args*/)throw() {}
//   template <class U> flexAllocator(const flexAllocator<U>& other);
//   T* allocate(std::size_t n)
//   {
//     return flexAlloc(n*sizeof(T));
//   }
//   void deallocate(T* p, std::size_t n)
//   {
//     // std::cout << "deallocate." << std::endl;
//     // flexFree((void*)p);
//   }
// };
// template <class T, class U>
// bool operator==(const flexAllocator<T>&, const flexAllocator<U>&);
// template <class T, class U>
// bool operator!=(const flexAllocator<T>&, const flexAllocator<U>&);


template <class T> 
struct flexAllocator {
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef T value_type;

    template <class U> struct rebind { typedef flexAllocator<U> other; };
    flexAllocator() throw() {}
    flexAllocator(const flexAllocator&) throw() {}

    template <class U> flexAllocator(const flexAllocator<U>&) throw(){}

    ~flexAllocator() throw() {}

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
bool operator==(const flexAllocator<T>&, const flexAllocator<U>&);
template <class T, class U>
bool operator!=(const flexAllocator<T>&, const flexAllocator<U>&);



// #include <flex.h>

// template <class T>
// class flexAllocator
// {
// public:
    // typedef size_t    size_type;
    // typedef ptrdiff_t difference_type;
    // typedef T*        pointer;
    // typedef const T*  const_pointer;
    // typedef T&        reference;
    // typedef const T&  const_reference;
    // typedef T         value_type;

    // flexAllocator() {}
    // flexAllocator(const flexAllocator&) {}
    // pointer   allocate(size_type n, const void * = 0) {
    //           T* t = (T*) flexAlloc(n * sizeof(T));
    //           return t;
    //         }
    // void      deallocate(void* p, size_type) {
    //           if (p) {
    //             flexFree(p);
    //           } 
    //         }

    // pointer           address(reference x) const { return &x; }
    // const_pointer     address(const_reference x) const { return &x; }
    // flexAllocator<T>&  operator=(const flexAllocator&) { return *this; }
    // void              construct(pointer p, const T& val) 
    //                 { new ((T*) p) T(val); }
    // void              destroy(pointer p) { flexFree(p); }

    // size_type         max_size() const { return size_t(-1); }

    // template <class U>
    // struct rebind { typedef flexAllocator<U> other; };

    // template <class U>
    // flexAllocator(const flexAllocator<U>&) {}

    // template <class U>
    // flexAllocator& operator=(const flexAllocator<U>&) { return *this; }
    // };

template <class T, class U>
bool operator==(const flexAllocator<T>&, const flexAllocator<U>&);
template <class T, class U>
bool operator!=(const flexAllocator<T>&, const flexAllocator<U>&);
