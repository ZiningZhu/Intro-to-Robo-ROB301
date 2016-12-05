ROB301 Project Report
===
### Outline
- Components
- Functionality Design
- Implementation
- Other Features

### Components  
This robot is consists of Lejos EV3 controller, three servo motors, one ultrasonic sensor, one color sensor, one gyro sensor, miscellaneous Lego bricks, and connectors.  

### Functionality Design
TODO


### Implementation
#### Steering
The steering of this robot depends on two differentially driven servo motors placed at the back. Two ports, `B` and `C`, controlls two servo motors separately. Several functionalities are needed in the steering of the robot, and their implementations are included.   
  - Steer with respect to a straight line  
  Set the speed of both motors to be the same, and rotate both motors in the same direction for the desired rotation angle. The desired angle is acquired by dividing the desired distance by an experimentally-determined constant, `dist_factor`, which happen to be $0.0463888$.
  ```
  Motor.B.setSpeed(BASE_SPEED);
  Motor.C.setSpeed(BASE_SPEED);
  Motor.B.rotate((int)(distance / dist_factor), true);
  Motor.C.rotate((int)(distance / dist_factor));
  ```
  - Turn for an angle  
  - Steer to a fixed position  

#### Estimate the current location  

#### Pizza Pick and Deliver  

#### Obstacle Avoidance
- Wall following
- Exception handling: when robot hits the boundary  

#### House Detection and Delivery
- Ultrasonic sensor
- Median Filter  
