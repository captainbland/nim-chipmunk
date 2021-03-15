type
  IdtypeT* {.size: sizeof(cint).} = enum
    P_ALL, P_PID, P_PGID


proc cpMessage*(condition: cstring; file: cstring; line: cint; isError: cint;
               isHardError: cint; message: cstring) {.varargs, cdecl,
    importc: "cpMessage", dynlib: "libchipmunk.so".}
type
  uint8T* = cuchar
  uint16T* = cushort
  uint32T* = cuint
  uint64T* = culong
  intLeast8T* = cchar
  intLeast16T* = cshort
  intLeast32T* = cint
  intLeast64T* = clong
  uintLeast8T* = cuchar
  uintLeast16T* = cushort
  uintLeast32T* = cuint
  uintLeast64T* = culong
  intFast8T* = cchar
  intFast16T* = clong
  intFast32T* = clong
  intFast64T* = clong
  uintFast8T* = cuchar
  uintFast16T* = culong
  uintFast32T* = culong
  uintFast64T* = culong
  intptrT* = clong
  uintptrT* = culong
  intmaxT* = cint
  uintmaxT* = cuint
  CpFloat* = cdouble

proc cpfmax*(a: CpFloat; b: CpFloat): CpFloat {.inline, cdecl.} =
  return if (a > b): a else: b

proc cpfmin*(a: CpFloat; b: CpFloat): CpFloat {.inline, cdecl.} =
  return if (a < b): a else: b

proc cpfabs*(f: CpFloat): CpFloat {.inline, cdecl.} =
  return if (f < 0): -f else: f

proc cpfclamp*(f: CpFloat; min: CpFloat; max: CpFloat): CpFloat {.inline, cdecl.} =
  return cpfmin(cpfmax(f, min), max)

proc cpfclamp01*(f: CpFloat): CpFloat {.inline, cdecl.} =
  return cpfmax(0.0, cpfmin(f, 1.0))

proc cpflerp*(f1: CpFloat; f2: CpFloat; t: CpFloat): CpFloat {.inline, cdecl.} =
  return f1 * (1.0 - t) + f2 * t

proc cpflerpconst*(f1: CpFloat; f2: CpFloat; d: CpFloat): CpFloat {.inline, cdecl.} =
  return f1 + cpfclamp(f2 - f1, -d, d)

type
  CpHashValue* = uintptrT
  CpCollisionID* = uint32T
  CpBool* = cuchar
  CpDataPointer* = pointer
  CpCollisionType* = uintptrT
  CpGroup* = uintptrT
  CpBitmask* = cuint
  CpTimestamp* = cuint


  CpTransform* {.bycopy.} = object
    a*: CpFloat
    b*: CpFloat
    c*: CpFloat
    d*: CpFloat
    tx*: CpFloat
    ty*: CpFloat

  CpMat2x2* {.bycopy.} = object
    a*: CpFloat
    b*: CpFloat
    c*: CpFloat
    d*: CpFloat

#   CpArray* {.bycopy.} = object
#   INNER_C_STRUCT_chipmunk_structs_77* {.bycopy.} = object
#   CpArbiterThread* {.bycopy.} = object
#   CpContact* {.bycopy.} = object
#   CpCollisionInfo* {.bycopy.} = object
#   CpCollisionHandler* {.bycopy.} = object
#   CpArbiter* {.bycopy.} = object
#   CpShapeMassInfo* {.bycopy.} = object
#   CpSegmentQueryInfo* {.bycopy.} = object
#   CpShapeFilter* {.bycopy.} = object
#   CpConstraintClass* {.bycopy.} = object
#   CpConstraint* {.bycopy.} = object
#   CpPinJoint* {.bycopy.} = object
#   CpSlideJoint* {.bycopy.} = object
#   CpPivotJoint* {.bycopy.} = object
#   CpGrooveJoint* {.bycopy.} = object
#   CpDampedSpring* {.bycopy.} = object
#   CpDampedRotarySpring* {.bycopy.} = object
#   CpRotaryLimitJoint* {.bycopy.} = object
#   CpRatchetJoint* {.bycopy.} = object
#   CpGearJoint* {.bycopy.} = object
#   CpSimpleMotor* {.bycopy.} = object
#   CpContactBufferHeader* = object
#   CpSpace* {.bycopy.} = object  
#   CpBody* {.bycopy.} = object
#   CpPostStepCallback* {.bycopy.} = object

# type
#   CpShapeClass* {.bycopy.} = object
#   CpShape* {.bycopy.} = object
#   CpCircleShape* {.bycopy.} = object
#   CpSegmentShape* {.bycopy.} = object
#   CpSplittingPlane* {.bycopy.} = object    



type

  CpVect* {.bycopy.} = object
    x*: CpFloat
    y*: CpFloat
  CpBodyType* {.size: sizeof(cint).} = enum
    CP_BODY_TYPE_DYNAMIC, CP_BODY_TYPE_KINEMATIC, CP_BODY_TYPE_STATIC
  CpBodyVelocityFunc* = proc (body: ptr CpBody; gravity: CpVect; damping: CpFloat;
                           dt: CpFloat) {.cdecl.}
  CpBodyPositionFunc* = proc (body: ptr CpBody; dt: CpFloat) {.cdecl.}
  CpArray* {.bycopy.} = object
    num*: cint
    max*: cint
    arr*: ptr pointer

  INNER_C_STRUCT_chipmunk_structs_77* {.bycopy.} = object
    root*: ptr CpBody
    next*: ptr CpBody
    idleTime*: CpFloat

  CpArbiterState* {.size: sizeof(cint).} = enum ##  Arbiter is active and its the first collision.
    CP_ARBITER_STATE_FIRST_COLLISION, ##  Arbiter is active and its not the first collision.
    CP_ARBITER_STATE_NORMAL,  ##  Collision has been explicitly ignored.
                            ##  Either by returning false from a begin collision handler or calling cpArbiterIgnore().
    CP_ARBITER_STATE_IGNORE,  ##  Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
    CP_ARBITER_STATE_CACHED,  ##  Collison arbiter is invalid because one of the shapes was removed.
    CP_ARBITER_STATE_INVALIDATED

  CpArbiterThread* {.bycopy.} = object
    next*: ptr CpArbiter
    prev*: ptr CpArbiter

  CpContact* {.bycopy.} = object
    r1*: CpVect
    r2*: CpVect
    nMass*: CpFloat
    tMass*: CpFloat
    bounce*: CpFloat           ##  TODO: look for an alternate bounce solution.
    jnAcc*: CpFloat
    jtAcc*: CpFloat
    jBias*: CpFloat
    bias*: CpFloat
    hash*: CpHashValue

  CpCollisionInfo* {.bycopy.} = object
    a*: ptr CpShape
    b*: ptr CpShape
    id*: CpCollisionID
    n*: CpVect
    count*: cint               ##  TODO Should this be a unique struct type?
    arr*: ptr CpContact

  CpCollisionHandler* {.bycopy.} = object
    typeA*: CpCollisionType
    typeB*: CpCollisionType
    beginFunc*: CpCollisionBeginFunc
    preSolveFunc*: CpCollisionPreSolveFunc
    postSolveFunc*: CpCollisionPostSolveFunc
    separateFunc*: CpCollisionSeparateFunc
    userData*: CpDataPointer

  CpArbiter* {.bycopy.} = object
    e*: CpFloat
    u*: CpFloat
    surfaceVr*: CpVect
    data*: CpDataPointer
    a*: ptr CpShape
    b*: ptr CpShape
    bodyA*: ptr CpBody
    bodyB*: ptr CpBody
    threadA*: CpArbiterThread
    threadB*: CpArbiterThread
    count*: cint
    contacts*: ptr CpContact
    n*: CpVect                 ##  Regular, wildcard A and wildcard B collision handlers.
    handler*: ptr CpCollisionHandler
    handlerA*: ptr CpCollisionHandler
    handlerB*: ptr CpCollisionHandler
    swapped*: CpBool
    stamp*: CpTimestamp
    state*: CpArbiterState

  CpShapeMassInfo* {.bycopy.} = object
    m*: CpFloat
    i*: CpFloat
    cog*: CpVect
    area*: CpFloat


  CpPointQueryInfo* {.bycopy.} = object
    shape*: ptr CpShape
    point*: CpVect
    distance*: CpFloat
    gradient*: CpVect

  CpSegmentQueryInfo* {.bycopy.} = object
    shape*: ptr CpShape
    point*: CpVect
    normal*: CpVect
    alpha*: CpFloat

  CpShapeFilter* {.bycopy.} = object
    group*: CpGroup
    categories*: CpBitmask
    mask*: CpBitmask

  CpShapeType* {.size: sizeof(cint).} = enum
    CIRCLE_SHAPE, SEGMENT_SHAPE, POLY_SHAPE, NUM_SHAPES
  CpShapeCacheDataImpl* = proc (shape: ptr CpShape; transform: CpTransform): CpBB {.cdecl.}
  CpShapeDestroyImpl* = proc (shape: ptr CpShape) {.cdecl.}
  CpShapePointQueryImpl* = proc (shape: ptr CpShape; p: CpVect;
                              info: ptr CpPointQueryInfo) {.cdecl.}
  CpShapeSegmentQueryImpl* = proc (shape: ptr CpShape; a: CpVect; b: CpVect;
                                radius: CpFloat; info: ptr CpSegmentQueryInfo) {.
      cdecl.}

  CpPolyShape* {.bycopy.} = object
    shape*: CpShape
    r*: CpFloat
    count*: cint               ##  The untransformed planes are appended at the end of the transformed planes.
    planes*: ptr CpSplittingPlane ##  Allocate a small number of splitting planes internally for simple poly.

  CpConstraintPreStepImpl* = proc (constraint: ptr CpConstraint; dt: CpFloat) {.cdecl.}
  CpConstraintApplyCachedImpulseImpl* = proc (constraint: ptr CpConstraint;
      dtCoef: CpFloat) {.cdecl.}
  CpConstraintApplyImpulseImpl* = proc (constraint: ptr CpConstraint; dt: CpFloat) {.
      cdecl.}
  CpConstraintGetImpulseImpl* = proc (constraint: ptr CpConstraint): CpFloat {.cdecl.}
  CpConstraintClass* {.bycopy.} = object
    preStep*: CpConstraintPreStepImpl
    applyCachedImpulse*: CpConstraintApplyCachedImpulseImpl
    applyImpulse*: CpConstraintApplyImpulseImpl
    getImpulse*: CpConstraintGetImpulseImpl

  CpConstraint* {.bycopy.} = object
    klass*: ptr CpConstraintClass
    space*: ptr CpSpace
    a*: ptr CpBody
    b*: ptr CpBody
    nextA*: ptr CpConstraint
    nextB*: ptr CpConstraint
    maxForce*: CpFloat
    errorBias*: CpFloat
    maxBias*: CpFloat
    collideBodies*: CpBool
    preSolve*: CpConstraintPreSolveFunc
    postSolve*: CpConstraintPostSolveFunc
    userData*: CpDataPointer

  CpPinJoint* {.bycopy.} = object
    constraint*: CpConstraint
    anchorA*: CpVect
    anchorB*: CpVect
    dist*: CpFloat
    r1*: CpVect
    r2*: CpVect
    n*: CpVect
    nMass*: CpFloat
    jnAcc*: CpFloat
    bias*: CpFloat

  CpSlideJoint* {.bycopy.} = object
    constraint*: CpConstraint
    anchorA*: CpVect
    anchorB*: CpVect
    min*: CpFloat
    max*: CpFloat
    r1*: CpVect
    r2*: CpVect
    n*: CpVect
    nMass*: CpFloat
    jnAcc*: CpFloat
    bias*: CpFloat

  CpPivotJoint* {.bycopy.} = object
    constraint*: CpConstraint
    anchorA*: CpVect
    anchorB*: CpVect
    r1*: CpVect
    r2*: CpVect
    k*: CpMat2x2
    jAcc*: CpVect
    bias*: CpVect

  CpDampedRotarySpringTorqueFunc* = proc (spring: ptr CpConstraint;
                                       relativeAngle: CpFloat): CpFloat {.cdecl.}

  CpDampedSpringForceFunc* = proc (spring: ptr CpConstraint; dist: CpFloat): CpFloat {.
      cdecl.}
  CpGrooveJoint* {.bycopy.} = object
    constraint*: CpConstraint
    grvN*: CpVect
    grvA*: CpVect
    grvB*: CpVect
    anchorB*: CpVect
    grvTn*: CpVect
    clamp*: CpFloat
    r1*: CpVect
    r2*: CpVect
    k*: CpMat2x2
    jAcc*: CpVect
    bias*: CpVect

  CpDampedSpring* {.bycopy.} = object
    constraint*: CpConstraint
    anchorA*: CpVect
    anchorB*: CpVect
    restLength*: CpFloat
    stiffness*: CpFloat
    damping*: CpFloat
    springForceFunc*: CpDampedSpringForceFunc
    targetVrn*: CpFloat
    vCoef*: CpFloat
    r1*: CpVect
    r2*: CpVect
    nMass*: CpFloat
    n*: CpVect
    jAcc*: CpFloat

  CpDampedRotarySpring* {.bycopy.} = object
    constraint*: CpConstraint
    restAngle*: CpFloat
    stiffness*: CpFloat
    damping*: CpFloat
    springTorqueFunc*: CpDampedRotarySpringTorqueFunc
    targetWrn*: CpFloat
    wCoef*: CpFloat
    iSum*: CpFloat
    jAcc*: CpFloat

  CpConstraintPreSolveFunc* = proc (constraint: ptr CpConstraint; space: ptr CpSpace) {.
      cdecl.}
  CpConstraintPostSolveFunc* = proc (constraint: ptr CpConstraint; space: ptr CpSpace) {.
      cdecl.}

  CpRotaryLimitJoint* {.bycopy.} = object
    constraint*: CpConstraint
    min*: CpFloat
    max*: CpFloat
    iSum*: CpFloat
    bias*: CpFloat
    jAcc*: CpFloat

  CpRatchetJoint* {.bycopy.} = object
    constraint*: CpConstraint
    angle*: CpFloat
    phase*: CpFloat
    ratchet*: CpFloat
    iSum*: CpFloat
    bias*: CpFloat
    jAcc*: CpFloat

  CpGearJoint* {.bycopy.} = object
    constraint*: CpConstraint
    phase*: CpFloat
    ratio*: CpFloat
    ratioInv*: CpFloat
    iSum*: CpFloat
    bias*: CpFloat
    jAcc*: CpFloat

  CpSimpleMotor* {.bycopy.} = object
    constraint*: CpConstraint
    rate*: CpFloat
    iSum*: CpFloat
    jAcc*: CpFloat

  CpSpaceArbiterApplyImpulseFunc* = proc (arb: ptr CpArbiter) {.cdecl.}
  CpSpace* {.bycopy.} = object
    iterations*: cint
    gravity*: CpVect
    damping*: CpFloat
    idleSpeedThreshold*: CpFloat
    sleepTimeThreshold*: CpFloat
    collisionSlop*: CpFloat
    collisionBias*: CpFloat
    collisionPersistence*: CpTimestamp
    userData*: CpDataPointer
    stamp*: CpTimestamp
    currDt*: CpFloat
    dynamicBodies*: ptr CpArray
    staticBodies*: ptr CpArray
    rousedBodies*: ptr CpArray
    sleepingComponents*: ptr CpArray
    shapeIDCounter*: CpHashValue
    staticShapes*: ptr CpSpatialIndex
    dynamicShapes*: ptr CpSpatialIndex
    constraints*: ptr CpArray
    arbiters*: ptr CpArray
    contactBuffersHead*: pointer
    cachedArbiters*: pointer
    pooledArbiters*: ptr CpArray
    allocatedBuffers*: ptr CpArray
    locked*: cuint
    usesWildcards*: CpBool
    collisionHandlers*: pointer
    defaultHandler*: CpCollisionHandler
    skipPostStep*: CpBool
    postStepCallbacks*: ptr CpArray
    staticBody*: ptr CpBody
  
  CpSpatialIndexBBFunc* = proc (obj: pointer): CpBB {.cdecl.}
  CpSpatialIndexIteratorFunc* = proc (obj: pointer; data: pointer) {.cdecl.}
  CpSpatialIndexQueryFunc* = proc (obj1: pointer; obj2: pointer; id: CpCollisionID;
                                data: pointer): CpCollisionID {.cdecl.}
  CpSpatialIndexSegmentQueryFunc* = proc (obj1: pointer; obj2: pointer; data: pointer): CpFloat {.
      cdecl.}
  CpSpatialIndex* {.bycopy.} = object
    klass*: pointer
    bbfunc*: CpSpatialIndexBBFunc
    staticIndex*: ptr CpSpatialIndex
    dynamicIndex*: ptr CpSpatialIndex

  CpCollisionBeginFunc* = proc (arb: ptr CpArbiter; space: ptr CpSpace;
                             userData: CpDataPointer): bool {.cdecl.}
  CpCollisionPreSolveFunc* = proc (arb: ptr CpArbiter; space: ptr CpSpace;
                                userData: CpDataPointer): bool {.cdecl.}
  CpCollisionPostSolveFunc* = proc (arb: ptr CpArbiter; space: ptr CpSpace;
                                 userData: CpDataPointer) {.cdecl.}
  CpCollisionSeparateFunc* = proc (arb: ptr CpArbiter; space: ptr CpSpace;
                                userData: CpDataPointer) {.cdecl.}

  CpBody* {.bycopy.} = object
    velocityFunc*: CpBodyVelocityFunc ##  Integration functions
    positionFunc*: CpBodyPositionFunc ##  mass and it's inverse
    m*: CpFloat
    mInv*: CpFloat             ##  moment of inertia and it's inverse
    i*: CpFloat
    iInv*: CpFloat             ##  center of gravity
    cog*: CpVect               ##  position, velocity, force
    p*: CpVect
    v*: CpVect
    f*: CpVect                 ##  Angle, angular velocity, torque (radians)
    a*: CpFloat
    w*: CpFloat
    t*: CpFloat
    transform*: CpTransform
    userData*: CpDataPointer   ##  "pseudo-velocities" used for eliminating overlap.
                           ##  Erin Catto has some papers that talk about what these are.
    vBias*: CpVect
    wBias*: CpFloat
    space*: ptr CpSpace
    shapeList*: ptr CpShape
    arbiterList*: ptr CpArbiter
    constraintList*: ptr CpConstraint
    sleeping*: INNER_C_STRUCT_chipmunk_structs_77

  CpPostStepFunc* = proc (space: ptr CpSpace; key: pointer; data: pointer) {.cdecl.}

  CpPostStepCallback* {.bycopy.} = object
    `func`*: CpPostStepFunc
    key*: pointer
    data*: pointer

  CpShapeClass* {.bycopy.} = object
    `type`*: CpShapeType
    cacheData*: CpShapeCacheDataImpl
    destroy*: CpShapeDestroyImpl
    pointQuery*: CpShapePointQueryImpl
    segmentQuery*: CpShapeSegmentQueryImpl

  CpShape* {.bycopy.} = object
    klass*: ptr CpShapeClass
    space*: ptr CpSpace
    body*: ptr CpBody
    massInfo*: CpShapeMassInfo
    bb*: CpBB
    sensor*: CpBool
    e*: CpFloat
    u*: CpFloat
    surfaceV*: CpVect
    userData*: CpDataPointer
    `type`*: CpCollisionType
    filter*: CpShapeFilter
    next*: ptr CpShape
    prev*: ptr CpShape
    hashid*: CpHashValue

  CpCircleShape* {.bycopy.} = object
    shape*: CpShape
    c*: CpVect
    tc*: CpVect
    r*: CpFloat

  CpSegmentShape* {.bycopy.} = object
    shape*: CpShape
    a*: CpVect
    b*: CpVect
    n*: CpVect
    ta*: CpVect
    tb*: CpVect
    tn*: CpVect
    r*: CpFloat
    aTangent*: CpVect
    bTangent*: CpVect

  CpSplittingPlane* {.bycopy.} = object
    v0*: CpVect
    n*: CpVect

  CpBB* {.bycopy.} = object
    l*: CpFloat
    b*: CpFloat
    r*: CpFloat
    t*: CpFloat


const
  CP_POLY_SHAPE_INLINE_ALLOC* = 6


 




# var cpvzero* {.importc: "cpvzero", dynlib: "libchipmunk.so".}: CpVect

proc cpv*(x: CpFloat; y: CpFloat): CpVect {.inline, cdecl.} =
  var v = CpVect(x:x, y:y)
  return v

proc cpveql*(v1: CpVect; v2: CpVect): bool {.inline, cdecl.} =
  return v1.x == v2.x and v1.y == v2.y

proc cpvadd*(v1: CpVect; v2: CpVect): CpVect {.inline, cdecl.} =
  return cpv(v1.x + v2.x, v1.y + v2.y)

proc cpvsub*(v1: CpVect; v2: CpVect): CpVect {.inline, cdecl.} =
  return cpv(v1.x - v2.x, v1.y - v2.y)

proc `-`*(v1: CpVect, v2: CpVect): CpVect {.inline.} =
    cpvsub(v1, v2)

proc cpvneg*(v: CpVect): CpVect {.inline, cdecl.} =
  return cpv(-v.x, -v.y)

proc cpvmult*(v: CpVect; s: CpFloat): CpVect {.inline, cdecl.} =
  return cpv(v.x * s, v.y * s)


proc cpvdot*(v1: CpVect; v2: CpVect): CpFloat {.inline, cdecl.} =
  return v1.x * v2.x + v1.y * v2.y

proc cpvcross*(v1: CpVect; v2: CpVect): CpFloat {.inline, cdecl.} =
  return v1.x * v2.y - v1.y * v2.x

proc cpvperp*(v: CpVect): CpVect {.inline, cdecl.} =
  return cpv(-v.y, v.x)

proc cpvrperp*(v: CpVect): CpVect {.inline, cdecl.} =
  return cpv(v.y, -v.x)

proc cpvproject*(v1: CpVect; v2: CpVect): CpVect {.inline, cdecl.} =
  return cpvmult(v2, cpvdot(v1, v2) / cpvdot(v2, v2))
import math
proc cpvforangle*(a: CpFloat): CpVect {.inline, cdecl.} =
  return cpv(cos(a), sin(a))

proc cpvtoangle*(v: CpVect): CpFloat {.inline, cdecl.} =
  return arctan2(v.y, v.x)

proc cpvrotate*(v1: CpVect; v2: CpVect): CpVect {.inline, cdecl.} =
  return cpv(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x)

proc cpvunrotate*(v1: CpVect; v2: CpVect): CpVect {.inline, cdecl.} =
  return cpv(v1.x * v2.x + v1.y * v2.y, v1.y * v2.x - v1.x * v2.y)

proc cpvlengthsq*(v: CpVect): CpFloat {.inline, cdecl.} =
  return cpvdot(v, v)

proc cpvlength*(v: CpVect): CpFloat {.inline, cdecl.} =
  return sqrt(cpvdot(v, v))

proc cpvlerp*(v1: CpVect; v2: CpVect; t: CpFloat): CpVect {.inline, cdecl.} =
  return cpvadd(cpvmult(v1, 1.0 - t), cpvmult(v2, t))

proc cpvnormalize*(v: CpVect): CpVect {.inline, cdecl.} =
  return cpvmult(v, 1.0 /
      (cpvlength(v) + (cast[cdouble](2.225073858507201e-308))))

proc cpvslerp*(v1: CpVect; v2: CpVect; t: CpFloat): CpVect {.inline, cdecl.} =
  var dot: CpFloat
  var omega: CpFloat
  if omega < 0.001:
    return cpvlerp(v1, v2, t)
  else:
    var denom: CpFloat
    return cpvadd(cpvmult(v1, sin((1.0 - t) * omega) * denom),
                 cpvmult(v2, sin(t * omega) * denom))

proc cpvslerpconst*(v1: CpVect; v2: CpVect; a: CpFloat): CpVect {.inline, cdecl.} =
  var dot: CpFloat
  var omega: CpFloat
  return cpvslerp(v1, v2, cpfmin(a, omega) / omega)

proc cpvclamp*(v: CpVect; len: CpFloat): CpVect {.inline, cdecl.} =
  return if (cpvdot(v, v) > len * len): cpvmult(cpvnormalize(v), len) else: v

proc cpvlerpconst*(v1: CpVect; v2: CpVect; d: CpFloat): CpVect {.inline, cdecl.} =
  return cpvadd(v1, cpvclamp(cpvsub(v2, v1), d))

proc cpvdist*(v1: CpVect; v2: CpVect): CpFloat {.inline, cdecl.} =
  return cpvlength(cpvsub(v1, v2))

proc cpvdistsq*(v1: CpVect; v2: CpVect): CpFloat {.inline, cdecl.} =
  return cpvlengthsq(cpvsub(v1, v2))

proc cpvnear*(v1: CpVect; v2: CpVect; dist: CpFloat): bool {.inline, cdecl.} =
  return cpvdistsq(v1, v2) < dist * dist

proc cpMat2x2New*(a: CpFloat; b: CpFloat; c: CpFloat; d: CpFloat): CpMat2x2 {.inline, cdecl.} =
  var m: CpMat2x2
  return m

proc cpMat2x2Transform*(m: CpMat2x2; v: CpVect): CpVect {.inline, cdecl.} =
  return cpv(v.x * m.a + v.y * m.b, v.x * m.c + v.y * m.d)



proc cpBBNew*(l: CpFloat; b: CpFloat; r: CpFloat; t: CpFloat): CpBB {.inline, cdecl.} =
  var bb: CpBB
  return bb

proc cpBBNewForExtents*(c: CpVect; hw: CpFloat; hh: CpFloat): CpBB {.inline, cdecl.} =
  return cpBBNew(c.x - hw, c.y - hh, c.x + hw, c.y + hh)

proc cpBBNewForCircle*(p: CpVect; r: CpFloat): CpBB {.inline, cdecl.} =
  return cpBBNewForExtents(p, r, r)

proc cpBBIntersects*(a: CpBB; b: CpBB): bool {.inline, cdecl.} =
  return a.l <= b.r and b.l <= a.r and a.b <= b.t and b.b <= a.t

proc cpBBContainsBB*(bb: CpBB; other: CpBB): bool {.inline, cdecl.} =
  return bb.l <= other.l and bb.r >= other.r and bb.b <= other.b and bb.t >= other.t

proc cpBBContainsVect*(bb: CpBB; v: CpVect): bool {.inline, cdecl.} =
  return bb.l <= v.x and bb.r >= v.x and bb.b <= v.y and bb.t >= v.y

proc cpBBMerge*(a: CpBB; b: CpBB): CpBB {.inline, cdecl.} =
  return cpBBNew(cpfmin(a.l, b.l), cpfmin(a.b, b.b), cpfmax(a.r, b.r), cpfmax(a.t, b.t))

proc cpBBExpand*(bb: CpBB; v: CpVect): CpBB {.inline, cdecl.} =
  return cpBBNew(cpfmin(bb.l, v.x), cpfmin(bb.b, v.y), cpfmax(bb.r, v.x),
                cpfmax(bb.t, v.y))

proc cpBBCenter*(bb: CpBB): CpVect {.inline, cdecl.} =
  return cpvlerp(cpv(bb.l, bb.b), cpv(bb.r, bb.t), 0.5)

proc cpBBArea*(bb: CpBB): CpFloat {.inline, cdecl.} =
  return (bb.r - bb.l) * (bb.t - bb.b)

proc cpBBMergedArea*(a: CpBB; b: CpBB): CpFloat {.inline, cdecl.} =
  return (cpfmax(a.r, b.r) - cpfmin(a.l, b.l)) *
      (cpfmax(a.t, b.t) - cpfmin(a.b, b.b))

proc cpBBSegmentQuery*(bb: CpBB; a: CpVect; b: CpVect): CpFloat {.inline, cdecl.} =
  discard

proc cpBBIntersectsSegment*(bb: CpBB; a: CpVect; b: CpVect): bool {.inline, cdecl.} =
  discard

proc cpBBClampVect*(bb: CpBB; v: CpVect): CpVect {.inline, cdecl.} =
  return cpv(cpfclamp(v.x, bb.l, bb.r), cpfclamp(v.y, bb.b, bb.t))

proc cpBBWrapVect*(bb: CpBB; v: CpVect): CpVect {.inline, cdecl.} =
  discard

proc cpBBOffset*(bb: CpBB; v: CpVect): CpBB {.inline, cdecl.} =
  return cpBBNew(bb.l + v.x, bb.b + v.y, bb.r + v.x, bb.t + v.y)

# var cpTransformIdentity* {.importc: "cpTransformIdentity", dynlib: "libchipmunk.so".}: CpTransform

proc cpTransformNew*(a: CpFloat; b: CpFloat; c: CpFloat; d: CpFloat; tx: CpFloat;
                    ty: CpFloat): CpTransform {.inline, cdecl.} =
  var t: CpTransform
  return t

proc cpTransformNewTranspose*(a: CpFloat; c: CpFloat; tx: CpFloat; b: CpFloat;
                             d: CpFloat; ty: CpFloat): CpTransform {.inline, cdecl.} =
  var t: CpTransform
  return t

proc cpTransformInverse*(t: CpTransform): CpTransform {.inline, cdecl.} =
  var invDet: CpFloat
  return cpTransformNewTranspose(t.d * invDet, -(t.c * invDet),
                                (t.c * t.ty - t.tx * t.d) * invDet, -(t.b * invDet),
                                t.a * invDet, (t.tx * t.b - t.a * t.ty) * invDet)

proc cpTransformMult*(t1: CpTransform; t2: CpTransform): CpTransform {.inline, cdecl.} =
  return cpTransformNewTranspose(t1.a * t2.a + t1.c * t2.b, t1.a * t2.c + t1.c * t2.d,
                                t1.a * t2.tx + t1.c * t2.ty + t1.tx,
                                t1.b * t2.a + t1.d * t2.b, t1.b * t2.c + t1.d * t2.d,
                                t1.b * t2.tx + t1.d * t2.ty + t1.ty)

proc cpTransformPoint*(t: CpTransform; p: CpVect): CpVect {.inline, cdecl.} =
  return cpv(t.a * p.x + t.c * p.y + t.tx, t.b * p.x + t.d * p.y + t.ty)

proc cpTransformVect*(t: CpTransform; v: CpVect): CpVect {.inline, cdecl.} =
  return cpv(t.a * v.x + t.c * v.y, t.b * v.x + t.d * v.y)

proc cpTransformbBB*(t: CpTransform; bb: CpBB): CpBB {.inline, cdecl.} =
  var center: CpVect
  var hw: CpFloat
  var hh: CpFloat
  var
    a: CpFloat
    b: CpFloat
    d: CpFloat
    e: CpFloat
  var hwMax: CpFloat
  var hhMax: CpFloat
  return cpBBNewForExtents(cpTransformPoint(t, center), hwMax, hhMax)

proc cpTransformTranslate*(translate: CpVect): CpTransform {.inline, cdecl.} =
  return cpTransformNewTranspose(1.0, 0.0, translate.x, 0.0, 1.0, translate.y)

proc cpTransformScale*(scaleX: CpFloat; scaleY: CpFloat): CpTransform {.inline, cdecl.} =
  return cpTransformNewTranspose(scaleX, 0.0, 0.0, 0.0, scaleY, 0.0)

proc cpTransformRotate*(radians: CpFloat): CpTransform {.inline, cdecl.} =
  var rot: CpVect
  return cpTransformNewTranspose(rot.x, -rot.y, 0.0, rot.y, rot.x, 0.0)

proc cpTransformRigid*(translate: CpVect; radians: CpFloat): CpTransform {.inline,
    cdecl.} =
  var rot: CpVect
  return cpTransformNewTranspose(rot.x, -rot.y, translate.x, rot.y, rot.x, translate.y)

proc cpTransformRigidInverse*(t: CpTransform): CpTransform {.inline, cdecl.} =
  return cpTransformNewTranspose(t.d, -t.c, (t.c * t.ty - t.tx * t.d), -t.b, t.a,
                                (t.tx * t.b - t.a * t.ty))

proc cpTransformWrap*(outer: CpTransform; inner: CpTransform): CpTransform {.inline,
    cdecl.} =
  return cpTransformMult(cpTransformInverse(outer), cpTransformMult(inner, outer))

proc cpTransformWrapInverse*(outer: CpTransform; inner: CpTransform): CpTransform {.
    inline, cdecl.} =
  return cpTransformMult(outer, cpTransformMult(inner, cpTransformInverse(outer)))

proc cpTransformOrtho*(bb: CpBB): CpTransform {.inline, cdecl.} =
  return cpTransformNewTranspose(2.0 / (bb.r - bb.l), 0.0,
                                -((bb.r + bb.l) / (bb.r - bb.l)), 0.0,
                                2.0 / (bb.t - bb.b),
                                -((bb.t + bb.b) / (bb.t - bb.b)))

proc cpTransformBoneScale*(v0: CpVect; v1: CpVect): CpTransform {.inline, cdecl.} =
  var d: CpVect
  return cpTransformNewTranspose(d.x, -d.y, v0.x, d.y, d.x, v0.y)

# proc cpTransformAxialScale*(axis: CpVect; pivot: CpVect; scale: CpFloat): CpTransform {.
#     inline, cdecl.} =
#   var A: CpFloat
#   var B: CpFloat
#   return cpTransformNewTranspose(scale * axis.x * axis.x + axis.y * axis.y, a, axis.x * b, a,
#                                 axis.x * axis.x + scale * axis.y * axis.y, axis.y * b)



# proc cpSpaceHashAlloc*(): ptr CpSpaceHash {.cdecl, importc: "cpSpaceHashAlloc",
#                                         dynlib: "libchipmunk.so".}
# proc cpSpaceHashInit*(hash: ptr CpSpaceHash; celldim: CpFloat; numcells: cint;
#                      bbfunc: CpSpatialIndexBBFunc; staticIndex: ptr CpSpatialIndex): ptr CpSpatialIndex {.
#     cdecl, importc: "cpSpaceHashInit", dynlib: "libchipmunk.so".}
# proc cpSpaceHashNew*(celldim: CpFloat; cells: cint; bbfunc: CpSpatialIndexBBFunc;
#                     staticIndex: ptr CpSpatialIndex): ptr CpSpatialIndex {.cdecl,
#     importc: "cpSpaceHashNew", dynlib: "libchipmunk.so".}
# proc cpSpaceHashResize*(hash: ptr CpSpaceHash; celldim: CpFloat; numcells: cint) {.
#     cdecl, importc: "cpSpaceHashResize", dynlib: "libchipmunk.so".}

# proc cpBBTreeAlloc*(): ptr CpBBTree {.cdecl, importc: "cpBBTreeAlloc",
#                                   dynlib: "libchipmunk.so".}
# proc cpBBTreeInit*(tree: ptr CpBBTree; bbfunc: CpSpatialIndexBBFunc;
#                   staticIndex: ptr CpSpatialIndex): ptr CpSpatialIndex {.cdecl,
#     importc: "cpBBTreeInit", dynlib: "libchipmunk.so".}
# proc cpBBTreeNew*(bbfunc: CpSpatialIndexBBFunc; staticIndex: ptr CpSpatialIndex): ptr CpSpatialIndex {.
#     cdecl, importc: "cpBBTreeNew", dynlib: "libchipmunk.so".}
# proc cpBBTreeOptimize*(index: ptr CpSpatialIndex) {.cdecl,
#     importc: "cpBBTreeOptimize", dynlib: "libchipmunk.so".}
# type
#   CpBBTreeVelocityFunc* = proc (obj: pointer): CpVect {.cdecl.}

# proc cpBBTreeSetVelocityFunc*(index: ptr CpSpatialIndex;
#                              `func`: CpBBTreeVelocityFunc) {.cdecl,
#     importc: "cpBBTreeSetVelocityFunc", dynlib: "libchipmunk.so".}


# proc cpSweep1DAlloc*(): ptr CpSweep1D {.cdecl, importc: "cpSweep1DAlloc",
#                                     dynlib: "libchipmunk.so".}
# proc cpSweep1DInit*(sweep: ptr CpSweep1D; bbfunc: CpSpatialIndexBBFunc;
#                    staticIndex: ptr CpSpatialIndex): ptr CpSpatialIndex {.cdecl,
#     importc: "cpSweep1DInit", dynlib: "libchipmunk.so".}
# proc cpSweep1DNew*(bbfunc: CpSpatialIndexBBFunc; staticIndex: ptr CpSpatialIndex): ptr CpSpatialIndex {.
#     cdecl, importc: "cpSweep1DNew", dynlib: "libchipmunk.so".}
# type
#   CpSpatialIndexDestroyImpl* = proc (index: ptr CpSpatialIndex) {.cdecl.}
#   CpSpatialIndexCountImpl* = proc (index: ptr CpSpatialIndex): cint {.cdecl.}
#   CpSpatialIndexEachImpl* = proc (index: ptr CpSpatialIndex;
#                                `func`: CpSpatialIndexIteratorFunc; data: pointer) {.
#       cdecl.}
#   CpSpatialIndexContainsImpl* = proc (index: ptr CpSpatialIndex; obj: pointer;
#                                    hashid: CpHashValue): bool {.cdecl.}
#   CpSpatialIndexInsertImpl* = proc (index: ptr CpSpatialIndex; obj: pointer;
#                                  hashid: CpHashValue) {.cdecl.}
#   CpSpatialIndexRemoveImpl* = proc (index: ptr CpSpatialIndex; obj: pointer;
#                                  hashid: CpHashValue) {.cdecl.}
#   CpSpatialIndexReindexImpl* = proc (index: ptr CpSpatialIndex) {.cdecl.}
#   CpSpatialIndexReindexObjectImpl* = proc (index: ptr CpSpatialIndex; obj: pointer;
#                                         hashid: CpHashValue) {.cdecl.}
#   CpSpatialIndexReindexQueryImpl* = proc (index: ptr CpSpatialIndex;
#                                        `func`: CpSpatialIndexQueryFunc;
#                                        data: pointer) {.cdecl.}
#   CpSpatialIndexQueryImpl* = proc (index: ptr CpSpatialIndex; obj: pointer; bb: CpBB;
#                                 `func`: CpSpatialIndexQueryFunc; data: pointer) {.
#       cdecl.}
#   CpSpatialIndexSegmentQueryImpl* = proc (index: ptr CpSpatialIndex; obj: pointer;
#                                        a: CpVect; b: CpVect; tExit: CpFloat;
#                                        `func`: CpSpatialIndexSegmentQueryFunc;
#                                        data: pointer) {.cdecl.}
#   CpSpatialIndexClass* {.bycopy.} = object
#     destroy*: CpSpatialIndexDestroyImpl
#     count*: CpSpatialIndexCountImpl
#     each*: CpSpatialIndexEachImpl
#     contains*: CpSpatialIndexContainsImpl
#     insert*: CpSpatialIndexInsertImpl
#     remove*: CpSpatialIndexRemoveImpl
#     reindex*: CpSpatialIndexReindexImpl
#     reindexObject*: CpSpatialIndexReindexObjectImpl
#     reindexQuery*: CpSpatialIndexReindexQueryImpl
#     query*: CpSpatialIndexQueryImpl
#     segmentQuery*: CpSpatialIndexSegmentQueryImpl


# proc cpSpatialIndexFree*(index: ptr CpSpatialIndex) {.cdecl,
#     importc: "cpSpatialIndexFree", dynlib: "libchipmunk.so".}
# proc cpSpatialIndexCollideStatic*(dynamicIndex: ptr CpSpatialIndex;
#                                  staticIndex: ptr CpSpatialIndex;
#                                  `func`: CpSpatialIndexQueryFunc; data: pointer) {.
#     cdecl, importc: "cpSpatialIndexCollideStatic", dynlib: "libchipmunk.so".}
# proc cpSpatialIndexDestroy*(index: ptr CpSpatialIndex) {.inline, cdecl.} =
#   if index.klass:
#     index.klass.destroy(index)

# proc cpSpatialIndexCount*(index: ptr CpSpatialIndex): cint {.inline, cdecl.} =
#   return index.klass.count(index)

# proc cpSpatialIndexEach*(index: ptr CpSpatialIndex;
#                         `func`: CpSpatialIndexIteratorFunc; data: pointer) {.inline,
#     cdecl.} =
#   index.klass.each(index, `func`, data)

# proc cpSpatialIndexContains*(index: ptr CpSpatialIndex; obj: pointer;
#                             hashid: CpHashValue): bool {.inline, cdecl.} =
#   return index.klass.contains(index, obj, hashid)

# proc cpSpatialIndexInsert*(index: ptr CpSpatialIndex; obj: pointer;
#                           hashid: CpHashValue) {.inline, cdecl.} =
#   index.klass.insert(index, obj, hashid)

# proc cpSpatialIndexRemove*(index: ptr CpSpatialIndex; obj: pointer;
#                           hashid: CpHashValue) {.inline, cdecl.} =
#   index.klass.remove(index, obj, hashid)

# proc cpSpatialIndexReindex*(index: ptr CpSpatialIndex) {.inline, cdecl.} =
#   index.klass.reindex(index)

# proc cpSpatialIndexReindexObject*(index: ptr CpSpatialIndex; obj: pointer;
#                                  hashid: CpHashValue) {.inline, cdecl.} =
#   index.klass.reindexObject(index, obj, hashid)

# proc cpSpatialIndexQuery*(index: ptr CpSpatialIndex; obj: pointer; bb: CpBB;
#                          `func`: CpSpatialIndexQueryFunc; data: pointer) {.inline,
#     cdecl.} =
#   index.klass.query(index, obj, bb, `func`, data)

# proc cpSpatialIndexSegmentQuery*(index: ptr CpSpatialIndex; obj: pointer; a: CpVect;
#                                 b: CpVect; tExit: CpFloat;
#                                 `func`: CpSpatialIndexSegmentQueryFunc;
#                                 data: pointer) {.inline, cdecl.} =
#   index.klass.segmentQuery(index, obj, a, b, tExit, `func`, data)

# proc cpSpatialIndexReindexQuery*(index: ptr CpSpatialIndex;
#                                 `func`: CpSpatialIndexQueryFunc; data: pointer) {.
#     inline, cdecl.} =
#   index.klass.reindexQuery(index, `func`, data)

# proc cpArbiterGetRestitution*(arb: ptr CpArbiter): CpFloat {.cdecl,
#     importc: "cpArbiterGetRestitution", dynlib: "libchipmunk.so".}
# proc cpArbiterSetRestitution*(arb: ptr CpArbiter; restitution: CpFloat) {.cdecl,
#     importc: "cpArbiterSetRestitution", dynlib: "libchipmunk.so".}
# proc cpArbiterGetFriction*(arb: ptr CpArbiter): CpFloat {.cdecl,
#     importc: "cpArbiterGetFriction", dynlib: "libchipmunk.so".}
# proc cpArbiterSetFriction*(arb: ptr CpArbiter; friction: CpFloat) {.cdecl,
#     importc: "cpArbiterSetFriction", dynlib: "libchipmunk.so".}
# proc cpArbiterGetSurfaceVelocity*(arb: ptr CpArbiter): CpVect {.cdecl,
#     importc: "cpArbiterGetSurfaceVelocity", dynlib: "libchipmunk.so".}
# proc cpArbiterSetSurfaceVelocity*(arb: ptr CpArbiter; vr: CpVect) {.cdecl,
#     importc: "cpArbiterSetSurfaceVelocity", dynlib: "libchipmunk.so".}
# proc cpArbiterGetUserData*(arb: ptr CpArbiter): CpDataPointer {.cdecl,
#     importc: "cpArbiterGetUserData", dynlib: "libchipmunk.so".}
# proc cpArbiterSetUserData*(arb: ptr CpArbiter; userData: CpDataPointer) {.cdecl,
#     importc: "cpArbiterSetUserData", dynlib: "libchipmunk.so".}
# proc cpArbiterTotalImpulse*(arb: ptr CpArbiter): CpVect {.cdecl,
#     importc: "cpArbiterTotalImpulse", dynlib: "libchipmunk.so".}
# proc cpArbiterTotalKE*(arb: ptr CpArbiter): CpFloat {.cdecl,
#     importc: "cpArbiterTotalKE", dynlib: "libchipmunk.so".}
# proc cpArbiterIgnore*(arb: ptr CpArbiter): bool {.cdecl, importc: "cpArbiterIgnore",
#     dynlib: "libchipmunk.so".}
proc cpArbiterGetShapes*(arb: ptr CpArbiter; a: ptr ptr CpShape; b: ptr ptr CpShape) {.
    cdecl, importc: "cpArbiterGetShapes", dynlib: "libchipmunk.so".}
proc cpArbiterGetBodies*(arb: ptr CpArbiter; a: ptr ptr CpBody; b: ptr ptr CpBody) {.cdecl,
    importc: "cpArbiterGetBodies", dynlib: "libchipmunk.so".}
# type
#   INNER_C_STRUCT_processed_chipmunk_1029* {.bycopy.} = object
#     pointA*: CpVect
#     pointB*: CpVect
#     distance*: CpFloat

#   CpContactPointSet* {.bycopy.} = object
#     count*: cint
#     normal*: CpVect
#     points*: array[2, INNER_C_STRUCT_processed_chipmunk_1029]


# proc cpArbiterGetContactPointSet*(arb: ptr CpArbiter): CpContactPointSet {.cdecl,
#     importc: "cpArbiterGetContactPointSet", dynlib: "libchipmunk.so".}
# proc cpArbiterSetContactPointSet*(arb: ptr CpArbiter; set: ptr CpContactPointSet) {.
#     cdecl, importc: "cpArbiterSetContactPointSet", dynlib: "libchipmunk.so".}
# proc cpArbiterIsFirstContact*(arb: ptr CpArbiter): bool {.cdecl,
#     importc: "cpArbiterIsFirstContact", dynlib: "libchipmunk.so".}
# proc cpArbiterIsRemoval*(arb: ptr CpArbiter): bool {.cdecl,
#     importc: "cpArbiterIsRemoval", dynlib: "libchipmunk.so".}
# proc cpArbiterGetCount*(arb: ptr CpArbiter): cint {.cdecl,
#     importc: "cpArbiterGetCount", dynlib: "libchipmunk.so".}
# proc cpArbiterGetNormal*(arb: ptr CpArbiter): CpVect {.cdecl,
#     importc: "cpArbiterGetNormal", dynlib: "libchipmunk.so".}
# proc cpArbiterGetPointA*(arb: ptr CpArbiter; i: cint): CpVect {.cdecl,
#     importc: "cpArbiterGetPointA", dynlib: "libchipmunk.so".}
# proc cpArbiterGetPointB*(arb: ptr CpArbiter; i: cint): CpVect {.cdecl,
#     importc: "cpArbiterGetPointB", dynlib: "libchipmunk.so".}
# proc cpArbiterGetDepth*(arb: ptr CpArbiter; i: cint): CpFloat {.cdecl,
#     importc: "cpArbiterGetDepth", dynlib: "libchipmunk.so".}
# proc cpArbiterCallWildcardBeginA*(arb: ptr CpArbiter; space: ptr CpSpace): bool {.
#     cdecl, importc: "cpArbiterCallWildcardBeginA", dynlib: "libchipmunk.so".}
# proc cpArbiterCallWildcardBeginB*(arb: ptr CpArbiter; space: ptr CpSpace): bool {.
#     cdecl, importc: "cpArbiterCallWildcardBeginB", dynlib: "libchipmunk.so".}
# proc cpArbiterCallWildcardPreSolveA*(arb: ptr CpArbiter; space: ptr CpSpace): bool {.
#     cdecl, importc: "cpArbiterCallWildcardPreSolveA", dynlib: "libchipmunk.so".}
# proc cpArbiterCallWildcardPreSolveB*(arb: ptr CpArbiter; space: ptr CpSpace): bool {.
#     cdecl, importc: "cpArbiterCallWildcardPreSolveB", dynlib: "libchipmunk.so".}
# proc cpArbiterCallWildcardPostSolveA*(arb: ptr CpArbiter; space: ptr CpSpace) {.cdecl,
#     importc: "cpArbiterCallWildcardPostSolveA", dynlib: "libchipmunk.so".}
# proc cpArbiterCallWildcardPostSolveB*(arb: ptr CpArbiter; space: ptr CpSpace) {.cdecl,
#     importc: "cpArbiterCallWildcardPostSolveB", dynlib: "libchipmunk.so".}
# proc cpArbiterCallWildcardSeparateA*(arb: ptr CpArbiter; space: ptr CpSpace) {.cdecl,
#     importc: "cpArbiterCallWildcardSeparateA", dynlib: "libchipmunk.so".}
# proc cpArbiterCallWildcardSeparateB*(arb: ptr CpArbiter; space: ptr CpSpace) {.cdecl,
#     importc: "cpArbiterCallWildcardSeparateB", dynlib: "libchipmunk.so".}



proc cpBodyAlloc*(): ptr CpBody {.cdecl, importc: "cpBodyAlloc", dynlib: "libchipmunk.so".}
proc cpBodyInit*(body: ptr CpBody; mass: CpFloat; moment: CpFloat): ptr CpBody {.cdecl,
    importc: "cpBodyInit", dynlib: "libchipmunk.so".}
proc cpBodyNew*(mass: CpFloat; moment: CpFloat): ptr CpBody {.cdecl,
    importc: "cpBodyNew", dynlib: "libchipmunk.so".}
proc cpBodyNewKinematic*(): ptr CpBody {.cdecl, importc: "cpBodyNewKinematic",
                                     dynlib: "libchipmunk.so".}
proc cpBodyNewStatic*(): ptr CpBody {.cdecl, importc: "cpBodyNewStatic",
                                  dynlib: "libchipmunk.so".}
proc cpBodyDestroy*(body: ptr CpBody) {.cdecl, importc: "cpBodyDestroy",
                                    dynlib: "libchipmunk.so".}
proc cpBodyFree*(body: ptr CpBody) {.cdecl, importc: "cpBodyFree",
                                 dynlib: "libchipmunk.so".}
proc cpBodyActivate*(body: ptr CpBody) {.cdecl, importc: "cpBodyActivate",
                                     dynlib: "libchipmunk.so".}
proc cpBodyActivateStatic*(body: ptr CpBody; filter: ptr CpShape) {.cdecl,
    importc: "cpBodyActivateStatic", dynlib: "libchipmunk.so".}
proc cpBodySleep*(body: ptr CpBody) {.cdecl, importc: "cpBodySleep",
                                  dynlib: "libchipmunk.so".}
proc cpBodySleepWithGroup*(body: ptr CpBody; group: ptr CpBody) {.cdecl,
    importc: "cpBodySleepWithGroup", dynlib: "libchipmunk.so".}
proc cpBodyIsSleeping*(body: ptr CpBody): bool {.cdecl, importc: "cpBodyIsSleeping",
    dynlib: "libchipmunk.so".}
proc cpBodyGetType*(body: ptr CpBody): CpBodyType {.cdecl, importc: "cpBodyGetType",
    dynlib: "libchipmunk.so".}
proc cpBodySetType*(body: ptr CpBody; `type`: CpBodyType) {.cdecl,
    importc: "cpBodySetType", dynlib: "libchipmunk.so".}
proc cpBodyGetSpace*(body: ptr CpBody): ptr CpSpace {.cdecl, importc: "cpBodyGetSpace",
    dynlib: "libchipmunk.so".}
proc cpBodyGetMass*(body: ptr CpBody): CpFloat {.cdecl, importc: "cpBodyGetMass",
    dynlib: "libchipmunk.so".}
proc cpBodySetMass*(body: ptr CpBody; m: CpFloat) {.cdecl, importc: "cpBodySetMass",
    dynlib: "libchipmunk.so".}
proc cpBodyGetMoment*(body: ptr CpBody): CpFloat {.cdecl, importc: "cpBodyGetMoment",
    dynlib: "libchipmunk.so".}
proc cpBodySetMoment*(body: ptr CpBody; i: CpFloat) {.cdecl,
    importc: "cpBodySetMoment", dynlib: "libchipmunk.so".}
proc cpBodyGetPosition*(body: ptr CpBody): CpVect {.cdecl,
    importc: "cpBodyGetPosition", dynlib: "libchipmunk.so".}
proc cpBodySetPosition*(body: ptr CpBody; pos: CpVect) {.cdecl,
    importc: "cpBodySetPosition", dynlib: "libchipmunk.so".}
proc cpBodyGetCenterOfGravity*(body: ptr CpBody): CpVect {.cdecl,
    importc: "cpBodyGetCenterOfGravity", dynlib: "libchipmunk.so".}
proc cpBodySetCenterOfGravity*(body: ptr CpBody; cog: CpVect) {.cdecl,
    importc: "cpBodySetCenterOfGravity", dynlib: "libchipmunk.so".}
proc cpBodyGetVelocity*(body: ptr CpBody): CpVect {.cdecl,
    importc: "cpBodyGetVelocity", dynlib: "libchipmunk.so".}
proc cpBodySetVelocity*(body: ptr CpBody; velocity: CpVect) {.cdecl,
    importc: "cpBodySetVelocity", dynlib: "libchipmunk.so".}
proc cpBodyGetForce*(body: ptr CpBody): CpVect {.cdecl, importc: "cpBodyGetForce",
    dynlib: "libchipmunk.so".}
proc cpBodySetForce*(body: ptr CpBody; force: CpVect) {.cdecl,
    importc: "cpBodySetForce", dynlib: "libchipmunk.so".}
proc cpBodyGetAngle*(body: ptr CpBody): CpFloat {.cdecl, importc: "cpBodyGetAngle",
    dynlib: "libchipmunk.so".}
proc cpBodySetAngle*(body: ptr CpBody; a: CpFloat) {.cdecl, importc: "cpBodySetAngle",
    dynlib: "libchipmunk.so".}
proc cpBodyGetAngularVelocity*(body: ptr CpBody): CpFloat {.cdecl,
    importc: "cpBodyGetAngularVelocity", dynlib: "libchipmunk.so".}
proc cpBodySetAngularVelocity*(body: ptr CpBody; angularVelocity: CpFloat) {.cdecl,
    importc: "cpBodySetAngularVelocity", dynlib: "libchipmunk.so".}
proc cpBodyGetTorque*(body: ptr CpBody): CpFloat {.cdecl, importc: "cpBodyGetTorque",
    dynlib: "libchipmunk.so".}
proc cpBodySetTorque*(body: ptr CpBody; torque: CpFloat) {.cdecl,
    importc: "cpBodySetTorque", dynlib: "libchipmunk.so".}
proc cpBodyGetRotation*(body: ptr CpBody): CpVect {.cdecl,
    importc: "cpBodyGetRotation", dynlib: "libchipmunk.so".}
proc cpBodyGetUserData*(body: ptr CpBody): CpDataPointer {.cdecl,
    importc: "cpBodyGetUserData", dynlib: "libchipmunk.so".}
proc cpBodySetUserData*(body: ptr CpBody; userData: CpDataPointer) {.cdecl,
    importc: "cpBodySetUserData", dynlib: "libchipmunk.so".}
proc cpBodySetVelocityUpdateFunc*(body: ptr CpBody; velocityFunc: CpBodyVelocityFunc) {.
    cdecl, importc: "cpBodySetVelocityUpdateFunc", dynlib: "libchipmunk.so".}
proc cpBodySetPositionUpdateFunc*(body: ptr CpBody; positionFunc: CpBodyPositionFunc) {.
    cdecl, importc: "cpBodySetPositionUpdateFunc", dynlib: "libchipmunk.so".}
proc cpBodyUpdateVelocity*(body: ptr CpBody; gravity: CpVect; damping: CpFloat;
                          dt: CpFloat) {.cdecl, importc: "cpBodyUpdateVelocity",
                                       dynlib: "libchipmunk.so".}
proc cpBodyUpdatePosition*(body: ptr CpBody; dt: CpFloat) {.cdecl,
    importc: "cpBodyUpdatePosition", dynlib: "libchipmunk.so".}
proc cpBodyLocalToWorld*(body: ptr CpBody; point: CpVect): CpVect {.cdecl,
    importc: "cpBodyLocalToWorld", dynlib: "libchipmunk.so".}
proc cpBodyWorldToLocal*(body: ptr CpBody; point: CpVect): CpVect {.cdecl,
    importc: "cpBodyWorldToLocal", dynlib: "libchipmunk.so".}
proc cpBodyApplyForceAtWorldPoint*(body: ptr CpBody; force: CpVect; point: CpVect) {.
    cdecl, importc: "cpBodyApplyForceAtWorldPoint", dynlib: "libchipmunk.so".}
proc cpBodyApplyForceAtLocalPoint*(body: ptr CpBody; force: CpVect; point: CpVect) {.
    cdecl, importc: "cpBodyApplyForceAtLocalPoint", dynlib: "libchipmunk.so".}
proc cpBodyApplyImpulseAtWorldPoint*(body: ptr CpBody; impulse: CpVect; point: CpVect) {.
    cdecl, importc: "cpBodyApplyImpulseAtWorldPoint", dynlib: "libchipmunk.so".}
proc cpBodyApplyImpulseAtLocalPoint*(body: ptr CpBody; impulse: CpVect; point: CpVect) {.
    cdecl, importc: "cpBodyApplyImpulseAtLocalPoint", dynlib: "libchipmunk.so".}
proc cpBodyGetVelocityAtWorldPoint*(body: ptr CpBody; point: CpVect): CpVect {.cdecl,
    importc: "cpBodyGetVelocityAtWorldPoint", dynlib: "libchipmunk.so".}
proc cpBodyGetVelocityAtLocalPoint*(body: ptr CpBody; point: CpVect): CpVect {.cdecl,
    importc: "cpBodyGetVelocityAtLocalPoint", dynlib: "libchipmunk.so".}
proc cpBodyKineticEnergy*(body: ptr CpBody): CpFloat {.cdecl,
    importc: "cpBodyKineticEnergy", dynlib: "libchipmunk.so".}
type
  CpBodyShapeIteratorFunc* = proc (body: ptr CpBody; shape: ptr CpShape; data: pointer) {.
      cdecl.}

proc cpBodyEachShape*(body: ptr CpBody; `func`: CpBodyShapeIteratorFunc; data: pointer) {.
    cdecl, importc: "cpBodyEachShape", dynlib: "libchipmunk.so".}
type
  CpBodyConstraintIteratorFunc* = proc (body: ptr CpBody;
                                     constraint: ptr CpConstraint; data: pointer) {.
      cdecl.}

proc cpBodyEachConstraint*(body: ptr CpBody; `func`: CpBodyConstraintIteratorFunc;
                          data: pointer) {.cdecl, importc: "cpBodyEachConstraint",
    dynlib: "libchipmunk.so".}
type
  CpBodyArbiterIteratorFunc* = proc (body: ptr CpBody; arbiter: ptr CpArbiter;
                                  data: pointer) {.cdecl.}

proc cpBodyEachArbiter*(body: ptr CpBody; `func`: CpBodyArbiterIteratorFunc;
                       data: pointer) {.cdecl, importc: "cpBodyEachArbiter",
                                      dynlib: "libchipmunk.so".}



# var CP_SHAPE_FILTER_ALL* {.importc: "CP_SHAPE_FILTER_ALL", dynlib: "libchipmunk.so".}: CpShapeFilter

# var CP_SHAPE_FILTER_NONE* {.importc: "CP_SHAPE_FILTER_NONE", dynlib: "libchipmunk.so".}: CpShapeFilter

proc cpShapeFilterNew*(group: CpGroup; categories: CpBitmask; mask: CpBitmask): CpShapeFilter {.
    inline, cdecl.} =
  var filter: CpShapeFilter
  return filter

proc cpShapeDestroy*(shape: ptr CpShape) {.cdecl, importc: "cpShapeDestroy",
                                       dynlib: "libchipmunk.so".}
proc cpShapeFree*(shape: ptr CpShape) {.cdecl, importc: "cpShapeFree",
                                    dynlib: "libchipmunk.so".}
proc cpShapeCacheBB*(shape: ptr CpShape): CpBB {.cdecl, importc: "cpShapeCacheBB",
    dynlib: "libchipmunk.so".}
proc cpShapeUpdate*(shape: ptr CpShape; transform: CpTransform): CpBB {.cdecl,
    importc: "cpShapeUpdate", dynlib: "libchipmunk.so".}
proc cpShapePointQuery*(shape: ptr CpShape; p: CpVect; `out`: ptr CpPointQueryInfo): CpFloat {.
    cdecl, importc: "cpShapePointQuery", dynlib: "libchipmunk.so".}
proc cpShapeSegmentQuery*(shape: ptr CpShape; a: CpVect; b: CpVect; radius: CpFloat;
                         info: ptr CpSegmentQueryInfo): bool {.cdecl,
    importc: "cpShapeSegmentQuery", dynlib: "libchipmunk.so".}
# proc cpShapesCollide*(a: ptr CpShape; b: ptr CpShape): CpContactPointSet {.cdecl,
#     importc: "cpShapesCollide", dynlib: "libchipmunk.so".}
proc cpShapeGetSpace*(shape: ptr CpShape): ptr CpSpace {.cdecl,
    importc: "cpShapeGetSpace", dynlib: "libchipmunk.so".}
proc cpShapeGetBody*(shape: ptr CpShape): ptr CpBody {.cdecl,
    importc: "cpShapeGetBody", dynlib: "libchipmunk.so".}
proc cpShapeSetBody*(shape: ptr CpShape; body: ptr CpBody) {.cdecl,
    importc: "cpShapeSetBody", dynlib: "libchipmunk.so".}
proc cpShapeGetMass*(shape: ptr CpShape): CpFloat {.cdecl, importc: "cpShapeGetMass",
    dynlib: "libchipmunk.so".}
proc cpShapeSetMass*(shape: ptr CpShape; mass: CpFloat) {.cdecl,
    importc: "cpShapeSetMass", dynlib: "libchipmunk.so".}
proc cpShapeGetDensity*(shape: ptr CpShape): CpFloat {.cdecl,
    importc: "cpShapeGetDensity", dynlib: "libchipmunk.so".}
proc cpShapeSetDensity*(shape: ptr CpShape; density: CpFloat) {.cdecl,
    importc: "cpShapeSetDensity", dynlib: "libchipmunk.so".}
proc cpShapeGetMoment*(shape: ptr CpShape): CpFloat {.cdecl,
    importc: "cpShapeGetMoment", dynlib: "libchipmunk.so".}
proc cpShapeGetArea*(shape: ptr CpShape): CpFloat {.cdecl, importc: "cpShapeGetArea",
    dynlib: "libchipmunk.so".}
proc cpShapeGetCenterOfGravity*(shape: ptr CpShape): CpVect {.cdecl,
    importc: "cpShapeGetCenterOfGravity", dynlib: "libchipmunk.so".}
proc cpShapeGetBB*(shape: ptr CpShape): CpBB {.cdecl, importc: "cpShapeGetBB",
    dynlib: "libchipmunk.so".}
proc cpShapeGetSensor*(shape: ptr CpShape): bool {.cdecl,
    importc: "cpShapeGetSensor", dynlib: "libchipmunk.so".}
proc cpShapeSetSensor*(shape: ptr CpShape; sensor: CpBool) {.cdecl,
    importc: "cpShapeSetSensor", dynlib: "libchipmunk.so".}
proc cpShapeGetElasticity*(shape: ptr CpShape): CpFloat {.cdecl,
    importc: "cpShapeGetElasticity", dynlib: "libchipmunk.so".}
proc cpShapeSetElasticity*(shape: ptr CpShape; elasticity: CpFloat) {.cdecl,
    importc: "cpShapeSetElasticity", dynlib: "libchipmunk.so".}
proc cpShapeGetFriction*(shape: ptr CpShape): CpFloat {.cdecl,
    importc: "cpShapeGetFriction", dynlib: "libchipmunk.so".}
proc cpShapeSetFriction*(shape: ptr CpShape; friction: CpFloat) {.cdecl,
    importc: "cpShapeSetFriction", dynlib: "libchipmunk.so".}
proc cpShapeGetSurfaceVelocity*(shape: ptr CpShape): CpVect {.cdecl,
    importc: "cpShapeGetSurfaceVelocity", dynlib: "libchipmunk.so".}
proc cpShapeSetSurfaceVelocity*(shape: ptr CpShape; surfaceVelocity: CpVect) {.cdecl,
    importc: "cpShapeSetSurfaceVelocity", dynlib: "libchipmunk.so".}
proc cpShapeGetUserData*(shape: ptr CpShape): CpDataPointer {.cdecl,
    importc: "cpShapeGetUserData", dynlib: "libchipmunk.so".}
proc cpShapeSetUserData*(shape: ptr CpShape; userData: CpDataPointer) {.cdecl,
    importc: "cpShapeSetUserData", dynlib: "libchipmunk.so".}
proc cpShapeGetCollisionType*(shape: ptr CpShape): CpCollisionType {.cdecl,
    importc: "cpShapeGetCollisionType", dynlib: "libchipmunk.so".}
proc cpShapeSetCollisionType*(shape: ptr CpShape; collisionType: CpCollisionType) {.
    cdecl, importc: "cpShapeSetCollisionType", dynlib: "libchipmunk.so".}
proc cpShapeGetFilter*(shape: ptr CpShape): CpShapeFilter {.cdecl,
    importc: "cpShapeGetFilter", dynlib: "libchipmunk.so".}
proc cpShapeSetFilter*(shape: ptr CpShape; filter: CpShapeFilter) {.cdecl,
    importc: "cpShapeSetFilter", dynlib: "libchipmunk.so".}
proc cpCircleShapeAlloc*(): ptr CpCircleShape {.cdecl, importc: "cpCircleShapeAlloc",
    dynlib: "libchipmunk.so".}
proc cpCircleShapeInit*(circle: ptr CpCircleShape; body: ptr CpBody; radius: CpFloat;
                       offset: CpVect): ptr CpCircleShape {.cdecl,
    importc: "cpCircleShapeInit", dynlib: "libchipmunk.so".}
proc cpCircleShapeNew*(body: ptr CpBody; radius: CpFloat; offset: CpVect): ptr CpShape {.
    cdecl, importc: "cpCircleShapeNew", dynlib: "libchipmunk.so".}
proc cpCircleShapeGetOffset*(shape: ptr CpShape): CpVect {.cdecl,
    importc: "cpCircleShapeGetOffset", dynlib: "libchipmunk.so".}
proc cpCircleShapeGetRadius*(shape: ptr CpShape): CpFloat {.cdecl,
    importc: "cpCircleShapeGetRadius", dynlib: "libchipmunk.so".}
proc cpSegmentShapeAlloc*(): ptr CpSegmentShape {.cdecl,
    importc: "cpSegmentShapeAlloc", dynlib: "libchipmunk.so".}
proc cpSegmentShapeInit*(seg: ptr CpSegmentShape; body: ptr CpBody; a: CpVect; b: CpVect;
                        radius: CpFloat): ptr CpSegmentShape {.cdecl,
    importc: "cpSegmentShapeInit", dynlib: "libchipmunk.so".}
proc cpSegmentShapeNew*(body: ptr CpBody; a: CpVect; b: CpVect; radius: CpFloat): ptr CpShape {.
    cdecl, importc: "cpSegmentShapeNew", dynlib: "libchipmunk.so".}
proc cpSegmentShapeSetNeighbors*(shape: ptr CpShape; prev: CpVect; next: CpVect) {.
    cdecl, importc: "cpSegmentShapeSetNeighbors", dynlib: "libchipmunk.so".}
proc cpSegmentShapeGetA*(shape: ptr CpShape): CpVect {.cdecl,
    importc: "cpSegmentShapeGetA", dynlib: "libchipmunk.so".}
proc cpSegmentShapeGetB*(shape: ptr CpShape): CpVect {.cdecl,
    importc: "cpSegmentShapeGetB", dynlib: "libchipmunk.so".}
proc cpSegmentShapeGetNormal*(shape: ptr CpShape): CpVect {.cdecl,
    importc: "cpSegmentShapeGetNormal", dynlib: "libchipmunk.so".}
proc cpSegmentShapeGetRadius*(shape: ptr CpShape): CpFloat {.cdecl,
    importc: "cpSegmentShapeGetRadius", dynlib: "libchipmunk.so".}
proc cpPolyShapeAlloc*(): ptr CpPolyShape {.cdecl, importc: "cpPolyShapeAlloc",
                                        dynlib: "libchipmunk.so".}
proc cpPolyShapeInit*(poly: ptr CpPolyShape; body: ptr CpBody; count: cint;
                     verts: ptr CpVect; transform: CpTransform; radius: CpFloat): ptr CpPolyShape {.
    cdecl, importc: "cpPolyShapeInit", dynlib: "libchipmunk.so".}
proc cpPolyShapeInitRaw*(poly: ptr CpPolyShape; body: ptr CpBody; count: cint;
                        verts: ptr CpVect; radius: CpFloat): ptr CpPolyShape {.cdecl,
    importc: "cpPolyShapeInitRaw", dynlib: "libchipmunk.so".}
proc cpPolyShapeNew*(body: ptr CpBody; count: cint; verts: ptr CpVect;
                    transform: CpTransform; radius: CpFloat): ptr CpShape {.cdecl,
    importc: "cpPolyShapeNew", dynlib: "libchipmunk.so".}
proc cpPolyShapeNewRaw*(body: ptr CpBody; count: cint; verts: ptr CpVect; radius: CpFloat): ptr CpShape {.
    cdecl, importc: "cpPolyShapeNewRaw", dynlib: "libchipmunk.so".}
proc cpBoxShapeInit*(poly: ptr CpPolyShape; body: ptr CpBody; width: CpFloat;
                    height: CpFloat; radius: CpFloat): ptr CpPolyShape {.cdecl,
    importc: "cpBoxShapeInit", dynlib: "libchipmunk.so".}
proc cpBoxShapeInit2*(poly: ptr CpPolyShape; body: ptr CpBody; box: CpBB; radius: CpFloat): ptr CpPolyShape {.
    cdecl, importc: "cpBoxShapeInit2", dynlib: "libchipmunk.so".}
proc cpBoxShapeNew*(body: ptr CpBody; width: CpFloat; height: CpFloat; radius: CpFloat): ptr CpShape {.
    cdecl, importc: "cpBoxShapeNew", dynlib: "libchipmunk.so".}
proc cpBoxShapeNew2*(body: ptr CpBody; box: CpBB; radius: CpFloat): ptr CpShape {.cdecl,
    importc: "cpBoxShapeNew2", dynlib: "libchipmunk.so".}
proc cpPolyShapeGetCount*(shape: ptr CpShape): cint {.cdecl,
    importc: "cpPolyShapeGetCount", dynlib: "libchipmunk.so".}
proc cpPolyShapeGetVert*(shape: ptr CpShape; index: cint): CpVect {.cdecl,
    importc: "cpPolyShapeGetVert", dynlib: "libchipmunk.so".}
proc cpPolyShapeGetRadius*(shape: ptr CpShape): CpFloat {.cdecl,
    importc: "cpPolyShapeGetRadius", dynlib: "libchipmunk.so".}

proc cpConstraintDestroy*(constraint: ptr CpConstraint) {.cdecl,
    importc: "cpConstraintDestroy", dynlib: "libchipmunk.so".}
proc cpConstraintFree*(constraint: ptr CpConstraint) {.cdecl,
    importc: "cpConstraintFree", dynlib: "libchipmunk.so".}
proc cpConstraintGetSpace*(constraint: ptr CpConstraint): ptr CpSpace {.cdecl,
    importc: "cpConstraintGetSpace", dynlib: "libchipmunk.so".}
proc cpConstraintGetBodyA*(constraint: ptr CpConstraint): ptr CpBody {.cdecl,
    importc: "cpConstraintGetBodyA", dynlib: "libchipmunk.so".}
proc cpConstraintGetBodyB*(constraint: ptr CpConstraint): ptr CpBody {.cdecl,
    importc: "cpConstraintGetBodyB", dynlib: "libchipmunk.so".}
proc cpConstraintGetMaxForce*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpConstraintGetMaxForce", dynlib: "libchipmunk.so".}
proc cpConstraintSetMaxForce*(constraint: ptr CpConstraint; maxForce: CpFloat) {.
    cdecl, importc: "cpConstraintSetMaxForce", dynlib: "libchipmunk.so".}
proc cpConstraintGetErrorBias*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpConstraintGetErrorBias", dynlib: "libchipmunk.so".}
proc cpConstraintSetErrorBias*(constraint: ptr CpConstraint; errorBias: CpFloat) {.
    cdecl, importc: "cpConstraintSetErrorBias", dynlib: "libchipmunk.so".}
proc cpConstraintGetMaxBias*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpConstraintGetMaxBias", dynlib: "libchipmunk.so".}
proc cpConstraintSetMaxBias*(constraint: ptr CpConstraint; maxBias: CpFloat) {.cdecl,
    importc: "cpConstraintSetMaxBias", dynlib: "libchipmunk.so".}
proc cpConstraintGetCollideBodies*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintGetCollideBodies", dynlib: "libchipmunk.so".}
proc cpConstraintSetCollideBodies*(constraint: ptr CpConstraint;
                                  collideBodies: CpBool) {.cdecl,
    importc: "cpConstraintSetCollideBodies", dynlib: "libchipmunk.so".}
proc cpConstraintGetPreSolveFunc*(constraint: ptr CpConstraint): CpConstraintPreSolveFunc {.
    cdecl, importc: "cpConstraintGetPreSolveFunc", dynlib: "libchipmunk.so".}
proc cpConstraintSetPreSolveFunc*(constraint: ptr CpConstraint;
                                 preSolveFunc: CpConstraintPreSolveFunc) {.cdecl,
    importc: "cpConstraintSetPreSolveFunc", dynlib: "libchipmunk.so".}
proc cpConstraintGetPostSolveFunc*(constraint: ptr CpConstraint): CpConstraintPostSolveFunc {.
    cdecl, importc: "cpConstraintGetPostSolveFunc", dynlib: "libchipmunk.so".}
proc cpConstraintSetPostSolveFunc*(constraint: ptr CpConstraint;
                                  postSolveFunc: CpConstraintPostSolveFunc) {.
    cdecl, importc: "cpConstraintSetPostSolveFunc", dynlib: "libchipmunk.so".}
proc cpConstraintGetUserData*(constraint: ptr CpConstraint): CpDataPointer {.cdecl,
    importc: "cpConstraintGetUserData", dynlib: "libchipmunk.so".}
proc cpConstraintSetUserData*(constraint: ptr CpConstraint; userData: CpDataPointer) {.
    cdecl, importc: "cpConstraintSetUserData", dynlib: "libchipmunk.so".}
proc cpConstraintGetImpulse*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpConstraintGetImpulse", dynlib: "libchipmunk.so".}
proc cpConstraintIsPinJoint*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsPinJoint", dynlib: "libchipmunk.so".}
proc cpPinJointAlloc*(): ptr CpPinJoint {.cdecl, importc: "cpPinJointAlloc",
                                      dynlib: "libchipmunk.so".}
proc cpPinJointInit*(joint: ptr CpPinJoint; a: ptr CpBody; b: ptr CpBody; anchorA: CpVect;
                    anchorB: CpVect): ptr CpPinJoint {.cdecl,
    importc: "cpPinJointInit", dynlib: "libchipmunk.so".}
proc cpPinJointNew*(a: ptr CpBody; b: ptr CpBody; anchorA: CpVect; anchorB: CpVect): ptr CpConstraint {.
    cdecl, importc: "cpPinJointNew", dynlib: "libchipmunk.so".}
proc cpPinJointGetAnchorA*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpPinJointGetAnchorA", dynlib: "libchipmunk.so".}
proc cpPinJointSetAnchorA*(constraint: ptr CpConstraint; anchorA: CpVect) {.cdecl,
    importc: "cpPinJointSetAnchorA", dynlib: "libchipmunk.so".}
proc cpPinJointGetAnchorB*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpPinJointGetAnchorB", dynlib: "libchipmunk.so".}
proc cpPinJointSetAnchorB*(constraint: ptr CpConstraint; anchorB: CpVect) {.cdecl,
    importc: "cpPinJointSetAnchorB", dynlib: "libchipmunk.so".}
proc cpPinJointGetDist*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpPinJointGetDist", dynlib: "libchipmunk.so".}
proc cpPinJointSetDist*(constraint: ptr CpConstraint; dist: CpFloat) {.cdecl,
    importc: "cpPinJointSetDist", dynlib: "libchipmunk.so".}
proc cpConstraintIsSlideJoint*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsSlideJoint", dynlib: "libchipmunk.so".}
proc cpSlideJointAlloc*(): ptr CpSlideJoint {.cdecl, importc: "cpSlideJointAlloc",
    dynlib: "libchipmunk.so".}
proc cpSlideJointInit*(joint: ptr CpSlideJoint; a: ptr CpBody; b: ptr CpBody;
                      anchorA: CpVect; anchorB: CpVect; min: CpFloat; max: CpFloat): ptr CpSlideJoint {.
    cdecl, importc: "cpSlideJointInit", dynlib: "libchipmunk.so".}
proc cpSlideJointNew*(a: ptr CpBody; b: ptr CpBody; anchorA: CpVect; anchorB: CpVect;
                     min: CpFloat; max: CpFloat): ptr CpConstraint {.cdecl,
    importc: "cpSlideJointNew", dynlib: "libchipmunk.so".}
proc cpSlideJointGetAnchorA*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpSlideJointGetAnchorA", dynlib: "libchipmunk.so".}
proc cpSlideJointSetAnchorA*(constraint: ptr CpConstraint; anchorA: CpVect) {.cdecl,
    importc: "cpSlideJointSetAnchorA", dynlib: "libchipmunk.so".}
proc cpSlideJointGetAnchorB*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpSlideJointGetAnchorB", dynlib: "libchipmunk.so".}
proc cpSlideJointSetAnchorB*(constraint: ptr CpConstraint; anchorB: CpVect) {.cdecl,
    importc: "cpSlideJointSetAnchorB", dynlib: "libchipmunk.so".}
proc cpSlideJointGetMin*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpSlideJointGetMin", dynlib: "libchipmunk.so".}
proc cpSlideJointSetMin*(constraint: ptr CpConstraint; min: CpFloat) {.cdecl,
    importc: "cpSlideJointSetMin", dynlib: "libchipmunk.so".}
proc cpSlideJointGetMax*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpSlideJointGetMax", dynlib: "libchipmunk.so".}
proc cpSlideJointSetMax*(constraint: ptr CpConstraint; max: CpFloat) {.cdecl,
    importc: "cpSlideJointSetMax", dynlib: "libchipmunk.so".}
proc cpConstraintIsPivotJoint*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsPivotJoint", dynlib: "libchipmunk.so".}
proc cpPivotJointAlloc*(): ptr CpPivotJoint {.cdecl, importc: "cpPivotJointAlloc",
    dynlib: "libchipmunk.so".}
proc cpPivotJointInit*(joint: ptr CpPivotJoint; a: ptr CpBody; b: ptr CpBody;
                      anchorA: CpVect; anchorB: CpVect): ptr CpPivotJoint {.cdecl,
    importc: "cpPivotJointInit", dynlib: "libchipmunk.so".}
proc cpPivotJointNew*(a: ptr CpBody; b: ptr CpBody; pivot: CpVect): ptr CpConstraint {.
    cdecl, importc: "cpPivotJointNew", dynlib: "libchipmunk.so".}
proc cpPivotJointNew2*(a: ptr CpBody; b: ptr CpBody; anchorA: CpVect; anchorB: CpVect): ptr CpConstraint {.
    cdecl, importc: "cpPivotJointNew2", dynlib: "libchipmunk.so".}
proc cpPivotJointGetAnchorA*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpPivotJointGetAnchorA", dynlib: "libchipmunk.so".}
proc cpPivotJointSetAnchorA*(constraint: ptr CpConstraint; anchorA: CpVect) {.cdecl,
    importc: "cpPivotJointSetAnchorA", dynlib: "libchipmunk.so".}
proc cpPivotJointGetAnchorB*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpPivotJointGetAnchorB", dynlib: "libchipmunk.so".}
proc cpPivotJointSetAnchorB*(constraint: ptr CpConstraint; anchorB: CpVect) {.cdecl,
    importc: "cpPivotJointSetAnchorB", dynlib: "libchipmunk.so".}
proc cpConstraintIsGrooveJoint*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsGrooveJoint", dynlib: "libchipmunk.so".}
proc cpGrooveJointAlloc*(): ptr CpGrooveJoint {.cdecl, importc: "cpGrooveJointAlloc",
    dynlib: "libchipmunk.so".}
proc cpGrooveJointInit*(joint: ptr CpGrooveJoint; a: ptr CpBody; b: ptr CpBody;
                       grooveA: CpVect; grooveB: CpVect; anchorB: CpVect): ptr CpGrooveJoint {.
    cdecl, importc: "cpGrooveJointInit", dynlib: "libchipmunk.so".}
proc cpGrooveJointNew*(a: ptr CpBody; b: ptr CpBody; grooveA: CpVect; grooveB: CpVect;
                      anchorB: CpVect): ptr CpConstraint {.cdecl,
    importc: "cpGrooveJointNew", dynlib: "libchipmunk.so".}
proc cpGrooveJointGetGrooveA*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpGrooveJointGetGrooveA", dynlib: "libchipmunk.so".}
proc cpGrooveJointSetGrooveA*(constraint: ptr CpConstraint; grooveA: CpVect) {.cdecl,
    importc: "cpGrooveJointSetGrooveA", dynlib: "libchipmunk.so".}
proc cpGrooveJointGetGrooveB*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpGrooveJointGetGrooveB", dynlib: "libchipmunk.so".}
proc cpGrooveJointSetGrooveB*(constraint: ptr CpConstraint; grooveB: CpVect) {.cdecl,
    importc: "cpGrooveJointSetGrooveB", dynlib: "libchipmunk.so".}
proc cpGrooveJointGetAnchorB*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpGrooveJointGetAnchorB", dynlib: "libchipmunk.so".}
proc cpGrooveJointSetAnchorB*(constraint: ptr CpConstraint; anchorB: CpVect) {.cdecl,
    importc: "cpGrooveJointSetAnchorB", dynlib: "libchipmunk.so".}
proc cpConstraintIsDampedSpring*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsDampedSpring", dynlib: "libchipmunk.so".}


proc cpDampedSpringAlloc*(): ptr CpDampedSpring {.cdecl,
    importc: "cpDampedSpringAlloc", dynlib: "libchipmunk.so".}
proc cpDampedSpringInit*(joint: ptr CpDampedSpring; a: ptr CpBody; b: ptr CpBody;
                        anchorA: CpVect; anchorB: CpVect; restLength: CpFloat;
                        stiffness: CpFloat; damping: CpFloat): ptr CpDampedSpring {.
    cdecl, importc: "cpDampedSpringInit", dynlib: "libchipmunk.so".}
proc cpDampedSpringNew*(a: ptr CpBody; b: ptr CpBody; anchorA: CpVect; anchorB: CpVect;
                       restLength: CpFloat; stiffness: CpFloat; damping: CpFloat): ptr CpConstraint {.
    cdecl, importc: "cpDampedSpringNew", dynlib: "libchipmunk.so".}
proc cpDampedSpringGetAnchorA*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpDampedSpringGetAnchorA", dynlib: "libchipmunk.so".}
proc cpDampedSpringSetAnchorA*(constraint: ptr CpConstraint; anchorA: CpVect) {.cdecl,
    importc: "cpDampedSpringSetAnchorA", dynlib: "libchipmunk.so".}
proc cpDampedSpringGetAnchorB*(constraint: ptr CpConstraint): CpVect {.cdecl,
    importc: "cpDampedSpringGetAnchorB", dynlib: "libchipmunk.so".}
proc cpDampedSpringSetAnchorB*(constraint: ptr CpConstraint; anchorB: CpVect) {.cdecl,
    importc: "cpDampedSpringSetAnchorB", dynlib: "libchipmunk.so".}
proc cpDampedSpringGetRestLength*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpDampedSpringGetRestLength", dynlib: "libchipmunk.so".}
proc cpDampedSpringSetRestLength*(constraint: ptr CpConstraint; restLength: CpFloat) {.
    cdecl, importc: "cpDampedSpringSetRestLength", dynlib: "libchipmunk.so".}
proc cpDampedSpringGetStiffness*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpDampedSpringGetStiffness", dynlib: "libchipmunk.so".}
proc cpDampedSpringSetStiffness*(constraint: ptr CpConstraint; stiffness: CpFloat) {.
    cdecl, importc: "cpDampedSpringSetStiffness", dynlib: "libchipmunk.so".}
proc cpDampedSpringGetDamping*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpDampedSpringGetDamping", dynlib: "libchipmunk.so".}
proc cpDampedSpringSetDamping*(constraint: ptr CpConstraint; damping: CpFloat) {.
    cdecl, importc: "cpDampedSpringSetDamping", dynlib: "libchipmunk.so".}
proc cpDampedSpringGetSpringForceFunc*(constraint: ptr CpConstraint): CpDampedSpringForceFunc {.
    cdecl, importc: "cpDampedSpringGetSpringForceFunc", dynlib: "libchipmunk.so".}
proc cpDampedSpringSetSpringForceFunc*(constraint: ptr CpConstraint;
                                      springForceFunc: CpDampedSpringForceFunc) {.
    cdecl, importc: "cpDampedSpringSetSpringForceFunc", dynlib: "libchipmunk.so".}
proc cpConstraintIsDampedRotarySpring*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsDampedRotarySpring", dynlib: "libchipmunk.so".}

proc cpDampedRotarySpringAlloc*(): ptr CpDampedRotarySpring {.cdecl,
    importc: "cpDampedRotarySpringAlloc", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringInit*(joint: ptr CpDampedRotarySpring; a: ptr CpBody;
                              b: ptr CpBody; restAngle: CpFloat; stiffness: CpFloat;
                              damping: CpFloat): ptr CpDampedRotarySpring {.cdecl,
    importc: "cpDampedRotarySpringInit", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringNew*(a: ptr CpBody; b: ptr CpBody; restAngle: CpFloat;
                             stiffness: CpFloat; damping: CpFloat): ptr CpConstraint {.
    cdecl, importc: "cpDampedRotarySpringNew", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringGetRestAngle*(constraint: ptr CpConstraint): CpFloat {.
    cdecl, importc: "cpDampedRotarySpringGetRestAngle", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringSetRestAngle*(constraint: ptr CpConstraint;
                                      restAngle: CpFloat) {.cdecl,
    importc: "cpDampedRotarySpringSetRestAngle", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringGetStiffness*(constraint: ptr CpConstraint): CpFloat {.
    cdecl, importc: "cpDampedRotarySpringGetStiffness", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringSetStiffness*(constraint: ptr CpConstraint;
                                      stiffness: CpFloat) {.cdecl,
    importc: "cpDampedRotarySpringSetStiffness", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringGetDamping*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpDampedRotarySpringGetDamping", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringSetDamping*(constraint: ptr CpConstraint; damping: CpFloat) {.
    cdecl, importc: "cpDampedRotarySpringSetDamping", dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringGetSpringTorqueFunc*(constraint: ptr CpConstraint): CpDampedRotarySpringTorqueFunc {.
    cdecl, importc: "cpDampedRotarySpringGetSpringTorqueFunc",
    dynlib: "libchipmunk.so".}
proc cpDampedRotarySpringSetSpringTorqueFunc*(constraint: ptr CpConstraint;
    springTorqueFunc: CpDampedRotarySpringTorqueFunc) {.cdecl,
    importc: "cpDampedRotarySpringSetSpringTorqueFunc", dynlib: "libchipmunk.so".}
proc cpConstraintIsRotaryLimitJoint*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsRotaryLimitJoint", dynlib: "libchipmunk.so".}
proc cpRotaryLimitJointAlloc*(): ptr CpRotaryLimitJoint {.cdecl,
    importc: "cpRotaryLimitJointAlloc", dynlib: "libchipmunk.so".}
proc cpRotaryLimitJointInit*(joint: ptr CpRotaryLimitJoint; a: ptr CpBody;
                            b: ptr CpBody; min: CpFloat; max: CpFloat): ptr CpRotaryLimitJoint {.
    cdecl, importc: "cpRotaryLimitJointInit", dynlib: "libchipmunk.so".}
proc cpRotaryLimitJointNew*(a: ptr CpBody; b: ptr CpBody; min: CpFloat; max: CpFloat): ptr CpConstraint {.
    cdecl, importc: "cpRotaryLimitJointNew", dynlib: "libchipmunk.so".}
proc cpRotaryLimitJointGetMin*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpRotaryLimitJointGetMin", dynlib: "libchipmunk.so".}
proc cpRotaryLimitJointSetMin*(constraint: ptr CpConstraint; min: CpFloat) {.cdecl,
    importc: "cpRotaryLimitJointSetMin", dynlib: "libchipmunk.so".}
proc cpRotaryLimitJointGetMax*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpRotaryLimitJointGetMax", dynlib: "libchipmunk.so".}
proc cpRotaryLimitJointSetMax*(constraint: ptr CpConstraint; max: CpFloat) {.cdecl,
    importc: "cpRotaryLimitJointSetMax", dynlib: "libchipmunk.so".}
proc cpConstraintIsRatchetJoint*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsRatchetJoint", dynlib: "libchipmunk.so".}
proc cpRatchetJointAlloc*(): ptr CpRatchetJoint {.cdecl,
    importc: "cpRatchetJointAlloc", dynlib: "libchipmunk.so".}
proc cpRatchetJointInit*(joint: ptr CpRatchetJoint; a: ptr CpBody; b: ptr CpBody;
                        phase: CpFloat; ratchet: CpFloat): ptr CpRatchetJoint {.cdecl,
    importc: "cpRatchetJointInit", dynlib: "libchipmunk.so".}
proc cpRatchetJointNew*(a: ptr CpBody; b: ptr CpBody; phase: CpFloat; ratchet: CpFloat): ptr CpConstraint {.
    cdecl, importc: "cpRatchetJointNew", dynlib: "libchipmunk.so".}
proc cpRatchetJointGetAngle*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpRatchetJointGetAngle", dynlib: "libchipmunk.so".}
proc cpRatchetJointSetAngle*(constraint: ptr CpConstraint; angle: CpFloat) {.cdecl,
    importc: "cpRatchetJointSetAngle", dynlib: "libchipmunk.so".}
proc cpRatchetJointGetPhase*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpRatchetJointGetPhase", dynlib: "libchipmunk.so".}
proc cpRatchetJointSetPhase*(constraint: ptr CpConstraint; phase: CpFloat) {.cdecl,
    importc: "cpRatchetJointSetPhase", dynlib: "libchipmunk.so".}
proc cpRatchetJointGetRatchet*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpRatchetJointGetRatchet", dynlib: "libchipmunk.so".}
proc cpRatchetJointSetRatchet*(constraint: ptr CpConstraint; ratchet: CpFloat) {.
    cdecl, importc: "cpRatchetJointSetRatchet", dynlib: "libchipmunk.so".}
proc cpConstraintIsGearJoint*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsGearJoint", dynlib: "libchipmunk.so".}
proc cpGearJointAlloc*(): ptr CpGearJoint {.cdecl, importc: "cpGearJointAlloc",
                                        dynlib: "libchipmunk.so".}
proc cpGearJointInit*(joint: ptr CpGearJoint; a: ptr CpBody; b: ptr CpBody;
                     phase: CpFloat; ratio: CpFloat): ptr CpGearJoint {.cdecl,
    importc: "cpGearJointInit", dynlib: "libchipmunk.so".}
proc cpGearJointNew*(a: ptr CpBody; b: ptr CpBody; phase: CpFloat; ratio: CpFloat): ptr CpConstraint {.
    cdecl, importc: "cpGearJointNew", dynlib: "libchipmunk.so".}
proc cpGearJointGetPhase*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpGearJointGetPhase", dynlib: "libchipmunk.so".}
proc cpGearJointSetPhase*(constraint: ptr CpConstraint; phase: CpFloat) {.cdecl,
    importc: "cpGearJointSetPhase", dynlib: "libchipmunk.so".}
proc cpGearJointGetRatio*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpGearJointGetRatio", dynlib: "libchipmunk.so".}
proc cpGearJointSetRatio*(constraint: ptr CpConstraint; ratio: CpFloat) {.cdecl,
    importc: "cpGearJointSetRatio", dynlib: "libchipmunk.so".}


proc cpConstraintIsSimpleMotor*(constraint: ptr CpConstraint): bool {.cdecl,
    importc: "cpConstraintIsSimpleMotor", dynlib: "libchipmunk.so".}
proc cpSimpleMotorAlloc*(): ptr CpSimpleMotor {.cdecl, importc: "cpSimpleMotorAlloc",
    dynlib: "libchipmunk.so".}
proc cpSimpleMotorInit*(joint: ptr CpSimpleMotor; a: ptr CpBody; b: ptr CpBody;
                       rate: CpFloat): ptr CpSimpleMotor {.cdecl,
    importc: "cpSimpleMotorInit", dynlib: "libchipmunk.so".}
proc cpSimpleMotorNew*(a: ptr CpBody; b: ptr CpBody; rate: CpFloat): ptr CpConstraint {.
    cdecl, importc: "cpSimpleMotorNew", dynlib: "libchipmunk.so".}
proc cpSimpleMotorGetRate*(constraint: ptr CpConstraint): CpFloat {.cdecl,
    importc: "cpSimpleMotorGetRate", dynlib: "libchipmunk.so".}
proc cpSimpleMotorSetRate*(constraint: ptr CpConstraint; rate: CpFloat) {.cdecl,
    importc: "cpSimpleMotorSetRate", dynlib: "libchipmunk.so".}


proc cpSpaceAlloc*(): ptr CpSpace {.cdecl, importc: "cpSpaceAlloc",
                                dynlib: "libchipmunk.so".}
proc cpSpaceInit*(space: ptr CpSpace): ptr CpSpace {.cdecl, importc: "cpSpaceInit",
    dynlib: "libchipmunk.so".}
proc cpSpaceNew*(): ptr CpSpace {.cdecl, importc: "cpSpaceNew", dynlib: "libchipmunk.so".}
proc cpSpaceDestroy*(space: ptr CpSpace) {.cdecl, importc: "cpSpaceDestroy",
                                       dynlib: "libchipmunk.so".}
proc cpSpaceFree*(space: ptr CpSpace) {.cdecl, importc: "cpSpaceFree",
                                    dynlib: "libchipmunk.so".}
proc cpSpaceGetIterations*(space: ptr CpSpace): cint {.cdecl,
    importc: "cpSpaceGetIterations", dynlib: "libchipmunk.so".}
proc cpSpaceSetIterations*(space: ptr CpSpace; iterations: cint) {.cdecl,
    importc: "cpSpaceSetIterations", dynlib: "libchipmunk.so".}
proc cpSpaceGetGravity*(space: ptr CpSpace): CpVect {.cdecl,
    importc: "cpSpaceGetGravity", dynlib: "libchipmunk.so".}
proc cpSpaceSetGravity*(space: ptr CpSpace; gravity: CpVect) {.cdecl,
    importc: "cpSpaceSetGravity", dynlib: "libchipmunk.so".}
proc cpSpaceGetDamping*(space: ptr CpSpace): CpFloat {.cdecl,
    importc: "cpSpaceGetDamping", dynlib: "libchipmunk.so".}
proc cpSpaceSetDamping*(space: ptr CpSpace; damping: CpFloat) {.cdecl,
    importc: "cpSpaceSetDamping", dynlib: "libchipmunk.so".}
proc cpSpaceGetIdleSpeedThreshold*(space: ptr CpSpace): CpFloat {.cdecl,
    importc: "cpSpaceGetIdleSpeedThreshold", dynlib: "libchipmunk.so".}
proc cpSpaceSetIdleSpeedThreshold*(space: ptr CpSpace; idleSpeedThreshold: CpFloat) {.
    cdecl, importc: "cpSpaceSetIdleSpeedThreshold", dynlib: "libchipmunk.so".}
proc cpSpaceGetSleepTimeThreshold*(space: ptr CpSpace): CpFloat {.cdecl,
    importc: "cpSpaceGetSleepTimeThreshold", dynlib: "libchipmunk.so".}
proc cpSpaceSetSleepTimeThreshold*(space: ptr CpSpace; sleepTimeThreshold: CpFloat) {.
    cdecl, importc: "cpSpaceSetSleepTimeThreshold", dynlib: "libchipmunk.so".}
proc cpSpaceGetCollisionSlop*(space: ptr CpSpace): CpFloat {.cdecl,
    importc: "cpSpaceGetCollisionSlop", dynlib: "libchipmunk.so".}
proc cpSpaceSetCollisionSlop*(space: ptr CpSpace; collisionSlop: CpFloat) {.cdecl,
    importc: "cpSpaceSetCollisionSlop", dynlib: "libchipmunk.so".}
proc cpSpaceGetCollisionBias*(space: ptr CpSpace): CpFloat {.cdecl,
    importc: "cpSpaceGetCollisionBias", dynlib: "libchipmunk.so".}
proc cpSpaceSetCollisionBias*(space: ptr CpSpace; collisionBias: CpFloat) {.cdecl,
    importc: "cpSpaceSetCollisionBias", dynlib: "libchipmunk.so".}
proc cpSpaceGetCollisionPersistence*(space: ptr CpSpace): CpTimestamp {.cdecl,
    importc: "cpSpaceGetCollisionPersistence", dynlib: "libchipmunk.so".}
proc cpSpaceSetCollisionPersistence*(space: ptr CpSpace;
                                    collisionPersistence: CpTimestamp) {.cdecl,
    importc: "cpSpaceSetCollisionPersistence", dynlib: "libchipmunk.so".}
proc cpSpaceGetUserData*(space: ptr CpSpace): CpDataPointer {.cdecl,
    importc: "cpSpaceGetUserData", dynlib: "libchipmunk.so".}
proc cpSpaceSetUserData*(space: ptr CpSpace; userData: CpDataPointer) {.cdecl,
    importc: "cpSpaceSetUserData", dynlib: "libchipmunk.so".}
proc cpSpaceGetStaticBody*(space: ptr CpSpace): ptr CpBody {.cdecl,
    importc: "cpSpaceGetStaticBody", dynlib: "libchipmunk.so".}
proc cpSpaceGetCurrentTimeStep*(space: ptr CpSpace): CpFloat {.cdecl,
    importc: "cpSpaceGetCurrentTimeStep", dynlib: "libchipmunk.so".}
proc cpSpaceIsLocked*(space: ptr CpSpace): bool {.cdecl, importc: "cpSpaceIsLocked",
    dynlib: "libchipmunk.so".}
proc cpSpaceAddDefaultCollisionHandler*(space: ptr CpSpace): ptr CpCollisionHandler {.
    cdecl, importc: "cpSpaceAddDefaultCollisionHandler", dynlib: "libchipmunk.so".}
proc cpSpaceAddCollisionHandler*(space: ptr CpSpace; a: CpCollisionType;
                                b: CpCollisionType): ptr CpCollisionHandler {.cdecl,
    importc: "cpSpaceAddCollisionHandler", dynlib: "libchipmunk.so".}
proc cpSpaceAddWildcardHandler*(space: ptr CpSpace; `type`: CpCollisionType): ptr CpCollisionHandler {.
    cdecl, importc: "cpSpaceAddWildcardHandler", dynlib: "libchipmunk.so".}
proc cpSpaceAddShape*(space: ptr CpSpace; shape: ptr CpShape): ptr CpShape {.cdecl,
    importc: "cpSpaceAddShape", dynlib: "libchipmunk.so".}
proc cpSpaceAddBody*(space: ptr CpSpace; body: ptr CpBody): ptr CpBody {.cdecl,
    importc: "cpSpaceAddBody", dynlib: "libchipmunk.so".}
proc cpSpaceAddConstraint*(space: ptr CpSpace; constraint: ptr CpConstraint): ptr CpConstraint {.
    cdecl, importc: "cpSpaceAddConstraint", dynlib: "libchipmunk.so".}
proc cpSpaceRemoveShape*(space: ptr CpSpace; shape: ptr CpShape) {.cdecl,
    importc: "cpSpaceRemoveShape", dynlib: "libchipmunk.so".}
proc cpSpaceRemoveBody*(space: ptr CpSpace; body: ptr CpBody) {.cdecl,
    importc: "cpSpaceRemoveBody", dynlib: "libchipmunk.so".}
proc cpSpaceRemoveConstraint*(space: ptr CpSpace; constraint: ptr CpConstraint) {.
    cdecl, importc: "cpSpaceRemoveConstraint", dynlib: "libchipmunk.so".}
proc cpSpaceContainsShape*(space: ptr CpSpace; shape: ptr CpShape): bool {.cdecl,
    importc: "cpSpaceContainsShape", dynlib: "libchipmunk.so".}
proc cpSpaceContainsBody*(space: ptr CpSpace; body: ptr CpBody): bool {.cdecl,
    importc: "cpSpaceContainsBody", dynlib: "libchipmunk.so".}
proc cpSpaceContainsConstraint*(space: ptr CpSpace; constraint: ptr CpConstraint): bool {.
    cdecl, importc: "cpSpaceContainsConstraint", dynlib: "libchipmunk.so".}

proc cpSpaceAddPostStepCallback*(space: ptr CpSpace; `func`: CpPostStepFunc;
                                key: pointer; data: pointer): bool {.cdecl,
    importc: "cpSpaceAddPostStepCallback", dynlib: "libchipmunk.so".}
type
  CpSpacePointQueryFunc* = proc (shape: ptr CpShape; point: CpVect; distance: CpFloat;
                              gradient: CpVect; data: pointer) {.cdecl.}

proc cpSpacePointQuery*(space: ptr CpSpace; point: CpVect; maxDistance: CpFloat;
                       filter: CpShapeFilter; `func`: CpSpacePointQueryFunc;
                       data: pointer) {.cdecl, importc: "cpSpacePointQuery",
                                      dynlib: "libchipmunk.so".}
proc cpSpacePointQueryNearest*(space: ptr CpSpace; point: CpVect;
                              maxDistance: CpFloat; filter: CpShapeFilter;
                              `out`: ptr CpPointQueryInfo): ptr CpShape {.cdecl,
    importc: "cpSpacePointQueryNearest", dynlib: "libchipmunk.so".}
type
  CpSpaceSegmentQueryFunc* = proc (shape: ptr CpShape; point: CpVect; normal: CpVect;
                                alpha: CpFloat; data: pointer) {.cdecl.}

proc cpSpaceSegmentQuery*(space: ptr CpSpace; start: CpVect; `end`: CpVect;
                         radius: CpFloat; filter: CpShapeFilter;
                         `func`: CpSpaceSegmentQueryFunc; data: pointer) {.cdecl,
    importc: "cpSpaceSegmentQuery", dynlib: "libchipmunk.so".}
proc cpSpaceSegmentQueryFirst*(space: ptr CpSpace; start: CpVect; `end`: CpVect;
                              radius: CpFloat; filter: CpShapeFilter;
                              `out`: ptr CpSegmentQueryInfo): ptr CpShape {.cdecl,
    importc: "cpSpaceSegmentQueryFirst", dynlib: "libchipmunk.so".}
type
  CpSpaceBBQueryFunc* = proc (shape: ptr CpShape; data: pointer) {.cdecl.}

proc cpSpaceBBQuery*(space: ptr CpSpace; bb: CpBB; filter: CpShapeFilter;
                    `func`: CpSpaceBBQueryFunc; data: pointer) {.cdecl,
    importc: "cpSpaceBBQuery", dynlib: "libchipmunk.so".}
# type
#   CpSpaceShapeQueryFunc* = proc (shape: ptr CpShape; points: ptr CpContactPointSet;
#                               data: pointer) {.cdecl.}

# proc cpSpaceShapeQuery*(space: ptr CpSpace; shape: ptr CpShape;
#                        `func`: CpSpaceShapeQueryFunc; data: pointer): bool {.cdecl,
#     importc: "cpSpaceShapeQuery", dynlib: "libchipmunk.so".}
type
  CpSpaceBodyIteratorFunc* = proc (body: ptr CpBody; data: pointer) {.cdecl.}

proc cpSpaceEachBody*(space: ptr CpSpace; `func`: CpSpaceBodyIteratorFunc;
                     data: pointer) {.cdecl, importc: "cpSpaceEachBody",
                                    dynlib: "libchipmunk.so".}
type
  CpSpaceShapeIteratorFunc* = proc (shape: ptr CpShape; data: pointer) {.cdecl.}

proc cpSpaceEachShape*(space: ptr CpSpace; `func`: CpSpaceShapeIteratorFunc;
                      data: pointer) {.cdecl, importc: "cpSpaceEachShape",
                                     dynlib: "libchipmunk.so".}
type
  CpSpaceConstraintIteratorFunc* = proc (constraint: ptr CpConstraint; data: pointer) {.
      cdecl.}

proc cpSpaceEachConstraint*(space: ptr CpSpace;
                           `func`: CpSpaceConstraintIteratorFunc; data: pointer) {.
    cdecl, importc: "cpSpaceEachConstraint", dynlib: "libchipmunk.so".}
proc cpSpaceReindexStatic*(space: ptr CpSpace) {.cdecl,
    importc: "cpSpaceReindexStatic", dynlib: "libchipmunk.so".}
proc cpSpaceReindexShape*(space: ptr CpSpace; shape: ptr CpShape) {.cdecl,
    importc: "cpSpaceReindexShape", dynlib: "libchipmunk.so".}
proc cpSpaceReindexShapesForBody*(space: ptr CpSpace; body: ptr CpBody) {.cdecl,
    importc: "cpSpaceReindexShapesForBody", dynlib: "libchipmunk.so".}
proc cpSpaceUseSpatialHash*(space: ptr CpSpace; dim: CpFloat; count: cint) {.cdecl,
    importc: "cpSpaceUseSpatialHash", dynlib: "libchipmunk.so".}
proc cpSpaceStep*(space: ptr CpSpace; dt: CpFloat) {.cdecl, importc: "cpSpaceStep",
    dynlib: "libchipmunk.so".}
type
  CpSpaceDebugColor* {.bycopy.} = object
    r*: cfloat
    g*: cfloat
    b*: cfloat
    a*: cfloat

  CpSpaceDebugDrawCircleImpl* = proc (pos: CpVect; angle: CpFloat; radius: CpFloat;
                                   outlineColor: CpSpaceDebugColor;
                                   fillColor: CpSpaceDebugColor;
                                   data: CpDataPointer) {.cdecl.}
  CpSpaceDebugDrawSegmentImpl* = proc (a: CpVect; b: CpVect; color: CpSpaceDebugColor;
                                    data: CpDataPointer) {.cdecl.}
  CpSpaceDebugDrawFatSegmentImpl* = proc (a: CpVect; b: CpVect; radius: CpFloat;
                                       outlineColor: CpSpaceDebugColor;
                                       fillColor: CpSpaceDebugColor;
                                       data: CpDataPointer) {.cdecl.}
  CpSpaceDebugDrawPolygonImpl* = proc (count: cint; verts: ptr CpVect; radius: CpFloat;
                                    outlineColor: CpSpaceDebugColor;
                                    fillColor: CpSpaceDebugColor;
                                    data: CpDataPointer) {.cdecl.}
  CpSpaceDebugDrawDotImpl* = proc (size: CpFloat; pos: CpVect;
                                color: CpSpaceDebugColor; data: CpDataPointer) {.
      cdecl.}
  CpSpaceDebugDrawColorForShapeImpl* = proc (shape: ptr CpShape; data: CpDataPointer): CpSpaceDebugColor {.
      cdecl.}
  CpSpaceDebugDrawFlags* {.size: sizeof(cint).} = enum
    CP_SPACE_DEBUG_DRAW_SHAPES = 1 shl 0, CP_SPACE_DEBUG_DRAW_CONSTRAINTS = 1 shl 1,
    CP_SPACE_DEBUG_DRAW_COLLISION_POINTS = 1 shl 2
  CpSpaceDebugDrawOptions* {.bycopy.} = object
    drawCircle*: CpSpaceDebugDrawCircleImpl
    drawSegment*: CpSpaceDebugDrawSegmentImpl
    drawFatSegment*: CpSpaceDebugDrawFatSegmentImpl
    drawPolygon*: CpSpaceDebugDrawPolygonImpl
    drawDot*: CpSpaceDebugDrawDotImpl
    flags*: CpSpaceDebugDrawFlags
    shapeOutlineColor*: CpSpaceDebugColor
    colorForShape*: CpSpaceDebugDrawColorForShapeImpl
    constraintColor*: CpSpaceDebugColor
    collisionPointColor*: CpSpaceDebugColor
    data*: CpDataPointer



proc cpSpaceDebugDraw*(space: ptr CpSpace; options: ptr CpSpaceDebugDrawOptions) {.
    cdecl, importc: "cpSpaceDebugDraw", dynlib: "libchipmunk.so".}
var cpVersionString* {.importc: "cpVersionString", dynlib: "libchipmunk.so".}: cstring

proc cpMomentForCircle*(m: CpFloat; r1: CpFloat; r2: CpFloat; offset: CpVect): CpFloat {.
    cdecl, importc: "cpMomentForCircle", dynlib: "libchipmunk.so".}
proc cpAreaForCircle*(r1: CpFloat; r2: CpFloat): CpFloat {.cdecl,
    importc: "cpAreaForCircle", dynlib: "libchipmunk.so".}
proc cpMomentForSegment*(m: CpFloat; a: CpVect; b: CpVect; radius: CpFloat): CpFloat {.
    cdecl, importc: "cpMomentForSegment", dynlib: "libchipmunk.so".}
proc cpAreaForSegment*(a: CpVect; b: CpVect; radius: CpFloat): CpFloat {.cdecl,
    importc: "cpAreaForSegment", dynlib: "libchipmunk.so".}
proc cpMomentForPoly*(m: CpFloat; count: cint; verts: ptr CpVect; offset: CpVect;
                     radius: CpFloat): CpFloat {.cdecl, importc: "cpMomentForPoly",
    dynlib: "libchipmunk.so".}
proc cpAreaForPoly*(count: cint; verts: ptr CpVect; radius: CpFloat): CpFloat {.cdecl,
    importc: "cpAreaForPoly", dynlib: "libchipmunk.so".}
proc cpCentroidForPoly*(count: cint; verts: ptr CpVect): CpVect {.cdecl,
    importc: "cpCentroidForPoly", dynlib: "libchipmunk.so".}
proc cpMomentForBox*(m: CpFloat; width: CpFloat; height: CpFloat): CpFloat {.cdecl,
    importc: "cpMomentForBox", dynlib: "libchipmunk.so".}
proc cpMomentForBox2*(m: CpFloat; box: CpBB): CpFloat {.cdecl,
    importc: "cpMomentForBox2", dynlib: "libchipmunk.so".}
proc cpConvexHull*(count: cint; verts: ptr CpVect; result: ptr CpVect; first: ptr cint;
                  tol: CpFloat): cint {.cdecl, importc: "cpConvexHull",
                                     dynlib: "libchipmunk.so".}
proc cpClosetPointOnSegment*(p: CpVect; a: CpVect; b: CpVect): CpVect {.inline, cdecl.} =
  var delta: CpVect
  var t: CpFloat
  return cpvadd(b, cpvmult(delta, t))
