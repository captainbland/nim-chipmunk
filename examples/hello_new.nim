import strutils
import ../src/chipmunk_new
import glm



proc v(x,y:float): CpVect = cpv(x,y)

var gravity = v(0, -100)
var vzero = v(0,0)
var space = cpSpaceNew()
space.cpSpaceSetGravity(gravity)
var ground_body = cpBodyNewStatic()
discard space.cpSpaceAddBody(ground_body)
var ground = cpSegmentShapeNew(ground_body, v(-20, 5), v(20, -5), 0)
#ground.cpShapeSetFriction(1.0f)
var discarded = space.cpSpaceAddShape(ground)

var radius = 5.0
var mass = 1.0

var moment = cpMomentForCircle(mass, 0, radius, vzero)

var ballBody = space.cpSpaceAddBody(cpBodyNew(mass, moment))
ballBody.cpBodySetPosition(v(1, 15))
echo ballBody.cpBodyGetPosition(), ",", ballBody.cpBodyGetPosition
var ballShape = space.cpSpaceAddShape(cpCircleShapeNew(ballBody, radius, vzero))
ballShape.cpShapeSetFriction(0.7)

var timeStep = 1.0/60.0

var time = 0.0
while time < 2:
  var pos = ballBody.cpBodyGetPosition()
  var vel = ballBody.cpBodyGetVelocity()
  echo "Time is $#. ballBody is at ($#, $#). Its velocity is ($#, $#)".format(
    time, pos.x, pos.y, vel.x, vel.y
  )

  space.cpSpaceStep(timeStep)

  time += timeStep

when defined chipmunkNoDestructors:
  ballShape.destroy()
  ballBody.destroy()
  ground.destroy()
  space.destroy()