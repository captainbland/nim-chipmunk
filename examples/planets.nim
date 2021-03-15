import
  chipmunk_new as chipmunk,
  csfml,
  math,
  random,
  glm,
  times

## Math's randomize
let now = getTime()
randomize(now.toUnix * 1_000_000_000 + now.nanosecond)

const
  gravityStrength = 50.float
  CTplanet = CpCollisionType(1)
  CTgravity = CpCollisionType(2)
  ScreenW = 640
  ScreenH = 480

## Global variables
var
  space = cpSpaceNew()
  window = newRenderWindow(
    videoMode(ScreenW, ScreenH, 32), "Planets demo", WindowStyle.Default
  )
  screenArea = IntRect(left: 20, top: 20, width: ScreenW-20, height: ScreenH-20)
  circleObjects: seq[ptr CpShape] = newSeq[ptr CpShape]()
  segmentObjects: seq[ptr CpShape] = newSeq[ptr CpShape]()
  running = true
  event: Event
  clock = newClock()

## Helper procedures
proc floor(vec: Vec2d): Vector2f =
  result.x = vec.x.floor
  result.y = vec.y.floor

proc cp2sfml(vec: chipmunk.CpVect): Vector2f =
  result.x = vec.x
  result.y = vec.y

proc initCircleShape(my_space: ptr CpSpace, my_shape: ptr CpShape,
                     userData: pointer = nil): ptr CpShape {.discardable.} =
  result = cpSpaceAddShape(my_space, my_shape)
  my_shape[].userData = csfml.newCircleShape(cpCircleShapeGetRadius(my_shape), 30)
  let circleData = cast[csfml.CircleShape](my_shape[].userData)
  circleData.origin = Vector2f(
    x:cpCircleShapeGetRadius(my_shape),
    y:cpCircleShapeGetRadius(my_shape)
  )
  circleData.fillColor = Green

proc drawCircle(win: RenderWindow, shape: ptr CpShape) =
    let circle = cast[csfml.CircleShape](shape.userData)
    circle.position = cp2sfml(shape.cpShapeGetBody().cpBodyGetPosition())
    win.draw(circle)

proc drawSegment(win: RenderWindow, shape: ptr chipmunk.CpShape) =
    win.draw(cast[csfml.VertexArray](shape.userData))

proc randomPoint(rect: var IntRect): CpVect =
  result.x = (random(rect.width) + rect.left).float
  result.y = (random(rect.height) + rect.top).float

proc addPlanet() =
  let
    mass = random(10_000)/10_000*10.0
    radius = mass * 2.0
    gravityRadius = radius * 8.8
    body = space.cpSpaceAddBody(cpBodyNew(mass, cpMomentForCircle(mass, 0.0, radius, cpv(0,0))))
    shape = initCircleShape(space, body.cpCircleShapeNew(radius, cpv(0,0)))
    gravity = initCircleShape(space, body.cpCircleShapeNew(gravityRadius, cpv(0,0)))
    gravityCircle = cast[csfml.CircleShape](gravity.userData)
  body.cpBodySetPosition randomPoint(screenArea)
  shape.cpShapeSetCollisionType CTplanet
  gravity.sensor = CpBool(true)
  gravity.cpShapeSetCollisionType CTgravity
  gravityCircle.fillColor = Transparent
  gravityCircle.outlineColor = Blue
  gravityCircle.outlineThickness = 2.0
  circleObjects.add(shape)
  circleObjects.add(gravity)

## Presolver callback procedure
## (Pre-Solve > collision happend, but has not been resolved yet)
## https://chipmunk-physics.net/release/Chipmunk-7.x/Chipmunk-7.0.1-Docs/#CollisionCallbacks
proc gravityApplicator(arb: ptr CpArbiter; space: CpSpace; data: pointer): bool {.cdecl.} =
  var
    bodyA: ptr CpBody
    bodyB: ptr CpBody
    dist: CpVect

  arb.cpArbiterGetBodies(addr bodyA, addr bodyB)
  
  dist = bodyA.cpBodyGetPosition() - bodyB.cpBodyGetPosition()
  bodyB.cpBodyApplyForceAtWorldPoint(
    dist.cpvMult(1.0 / dist.cpvnormalize().cpvlength * gravityStrength),
    cpv(0,0)
  )


## Startup initialization
window.frameRateLimit = 60
space.iterations = 20
## Add the collision callback to the presolver
var handler = space.cpSpaceAddCollisionHandler(CTgravity, CTplanet)
handler.preSolveFunc = cast[CpCollisionPreSolveFunc](gravityApplicator)

## Add the planets and the borders
block:
  let borders = [CpVect(x:0, y:0), CpVect(x:0, y:ScreenH),
                 CpVect(x:ScreenW, y:ScreenH), CpVect(x:ScreenW, y:0)]
  for i in 0..3:
    var newSegment = space.cpSpaceAddShape(space.staticBody.cpSegmentShapeNew(
      borders[i], borders[(i + 1) mod 4], 16.0)
    )
    newSegment.userData = csfml.newVertexArray(PrimitiveType.Lines, 2)
    let vertexData = cast[VertexArray](newSegment.userData)
    vertexData.getVertex(0).position = cast[ptr CpSegmentShape](newSegment).a.cp2sfml()
    vertexData.getVertex(1).position = cast[ptr CpSegmentShape](newSegment).b.cp2sfml()
    vertexData.getVertex(0).color = Blue
    vertexData.getVertex(1).color = Blue
    segmentObjects.add(newSegment)
  for i in 0..29:
    addPlanet()

## Main loop
while running:
  while window.pollEvent(event):
    if event.kind == EventType.Closed:
      running = false
      break
    elif event.kind == EventType.KeyPressed:
      if event.key.code == KeyCode.Escape:
        running = false
        break

  let dt = clock.restart.asSeconds / 100

  space.cpSpaceStep(dt)
  window.clear(Black)
  for obj in circleObjects:
    window.drawCircle(obj)
  for obj in segmentObjects:
    window.drawSegment(obj)
  window.display()

## Cleanup
space.cpSpaceDestroy()
