# SCARA Robot Arm README // ENGR 113

**moveMotors** is a function that coordinates the simultaneous movement of all three stepper motors;

**degToSteps** converts degrees to steps understood by the moveMotors function;

**zToSteps** converts vertical distance (in cm) to steps understood by the moveMotors function;

**inverseKinematics** accepts an input of a desired XY location and the lengths L1 and L2 (the lengths from the base pivot axis to the joint pivot axis, and then the joint pivot axis to the end effector,in cm in this case) and returns angles of rotation for the stepper motors;

**moveTo** accepts an input of a desired XY location and the constant lengths L1 and L2, as well as the angles of rotation from inverseKinematics, and calls moveMotors and degToSteps to move the end effector to the desired XY location;

**setup** sets the initial speed of the stepper motors, and calls moveTo to bring the robot to a predetermined starting point;

**loop** runs the two solutions required for the lab demonstration on repeat, with a short delay between.

## Electronic Locking
Over-rotation can introduce unnecessary stress on mechanical and wiring components, as well as calibration issues that may throw off the precision of the robot arm. As a result, both planar rotation stepper motors have been limited to a rotational freedom of ±135°.
The **moveMotors** function checks if the current motor1_position and motor2_position is greater than or equal to -768 steps (-135°) and less than or equal to 768 steps (135°), and checks that the next_position of motor 3 is within bounds before executing the movement.


