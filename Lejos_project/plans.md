Plan
===
### Algorithm
(1) Pick up pizza, determined by the key press  
(2) Then go back, start recording and follow blue boundary line.  
(3) Go to either one of blue, red, and green route. Calculate the distance.  
(4) Teach and repeat. Come back.

### A better idea
(1) Pick the pizza.  
(2) Go directly into the obstacles. Use sensor on the side to detect, so that the car can go around the obstacle.  
(3) Meanwhile, use the odometer in each motor to calculate the current location of the robot.  
[tutorial](https://view.officeapps.live.com/op/view.aspx?src=http://www.cs.scranton.edu/~bi/2015s-html/cs358/EV3-Motor-Guide.docx)
[doc](http://www.lejos.org/ev3/docs/lejos/hardware/motor/BaseRegulatedMotor.html)


### Hardware requirement:
- 2 servo motors for steering  
- 2 servo motors to pick up pizza  
- 1 color sensor facing down, to keep track of the route  
- 1 ultrasound sensor facing front, detecting obstacles  
- Total 6 ports utilized.  



### Data
齿轮比：
  最小的是8  
  黄色的是12  
  四孔灰色24  
  Grip的齿轮比改了

 Port 1: Ultrasounic
 Port 2: Infrared
 Port 3: Gyro

 Port A: Grip Motor
 Port B: Left Motor
 Port C: Right Motor
