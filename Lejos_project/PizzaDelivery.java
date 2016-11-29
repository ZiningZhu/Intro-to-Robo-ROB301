import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import java.lang.Math;
import java.util.ArrayList;
import java.util.*;

public class PizzaDelivery {
    private static final int BASE_SPEED = 120;
    // Base speed for motors.

    private static final double l = 0.0463888;
    // motor rotation factor. (length in cm) / l = motor rotation angle
    private static final int RIGHT_ANGLE = 210;
    // both motor shall rotate this angle in opposite direction at the same time
    // to achieve a right angle turn of the robot
    private static final int ROBOT_LENGTH = 10;
    // in centimeters
    private static final int GRIP_ANGLE = 80;
    // Angle the motor has to rotate to close the grip

    private static double x = 0;
    private static double y = 0;
    // Coordinates of the robot, at any given time point. In centimeters
    // To be updated in steerToColor() and pivotAndDeliverPizza() using a modified unicycle model.

    private static EV3UltrasonicSensor sonic;
    private static EV3ColorSensor color;
    private static EV3GyroSensor tilt;
    // Three sensors, corresponding to our three sensor ports


    private static double GYRO_OFFSET = 0;

    private static void initialize(){
        // Initialize the sensors
        sonic = new EV3UltrasonicSensor(SensorPort.S1);
        color = new EV3ColorSensor(SensorPort.S2);
        tilt = new EV3GyroSensor(SensorPort.S3);

        // Reset the Motor encoders
        Motor.A.resetTachoCount();
        Motor.B.resetTachoCount();
        Motor.C.resetTachoCount();

        // Offset the initial error of gyro sensor
        int tiltSampleSize = tilt.sampleSize();
        float[] tiltSample = new float[tiltSampleSize];
        tilt.getAngleMode().fetchSample(tiltSample, 0);
        GYRO_OFFSET = tiltSample[0];


    }
    private static char[] getInputInterface() {
        char pizzachoice, circlechoice, houseside;
        int housenumber;
        LCD.clear();
        System.out.println("Press ENTER to start selection...");
        while(!Button.ENTER.isDown()) ;
        while(Button.ENTER.isDown()) ;

        LCD.clear();
        System.out.println("Select Pizza location: LEFT/RIGHT");
        while ((!Button.LEFT.isDown()) && (!Button.RIGHT.isDown())) ;
        if (Button.LEFT.isDown()) pizzachoice = 0;
        else pizzachoice = 1;
        while ((Button.LEFT.isDown()) || (Button.RIGHT.isDown())) ;
        System.out.println("Pizza selected" + (pizzachoice==0?" left":" right"));

        LCD.clear();
        System.out.println("Select circle: LEFT/UP/RIGHT");
        while ((!Button.LEFT.isDown()) && (!Button.UP.isDown()) && (!Button.RIGHT.isDown())) ;
        String selectedCircleName = "";
        if (Button.LEFT.isDown()) {
            circlechoice = 0;
            selectedCircleName = " Green";
        } else if (Button.UP.isDown()) {
            circlechoice = 1;
            selectedCircleName = " Blue";
        } else {
            circlechoice = 2;
            selectedCircleName = " Red";
        }
        while ((Button.LEFT.isDown()) || (Button.UP.isDown()) || (Button.RIGHT.isDown())) ;
        System.out.println("Circle selected: " + selectedCircleName);

        LCD.clear();
        System.out.println("Select house side: LEFT/RIGHT");
        while ((!Button.LEFT.isDown()) && (!Button.RIGHT.isDown())) ;
        if (Button.LEFT.isDown()) houseside = 0;
        else houseside = 1;
        while ((Button.LEFT.isDown()) || (Button.RIGHT.isDown())) ;
        System.out.println("House side selected" + (houseside==0?" left":" right"));

        LCD.clear();
        System.out.println("Select house number: UP+/DOWN-, ENTER confirm");
        housenumber = 0;
        System.out.println("Current number: " + housenumber);
        boolean confirmed = false;
        while (!confirmed) {
            if (Button.ENTER.isDown()) {
                confirmed = true;
                while (Button.ENTER.isDown()) ;
                break;
            }
            if (Button.UP.isDown()) {
                housenumber += 1;
                System.out.println("Curr number: " + housenumber);
                while (Button.UP.isDown()) ;
            }
            if (Button.DOWN.isDown()) {
                housenumber -= 1;
                System.out.println("Curr number: " + housenumber);
                while (Button.DOWN.isDown()) ;
            }
        }
        System.out.println("Housenumber confirmed: " + housenumber);
        while(!Button.ENTER.isDown()){
        	while(Button.ENTER.isDown());
        }
        char[] res = new char[4];
        res[0] = pizzachoice;
        res[1] = circlechoice;
        res[2] = houseside;
        res[3] = (char)housenumber;

        return res;
    }


    private static void rotateRobot(double angle) {
    	// angle<0 then turn right;
    	// angle>0 then turn left
    	angle = angle * 0.95;
        Motor.B.setSpeed(BASE_SPEED/2);
        Motor.C.setSpeed(BASE_SPEED/2);
        int tiltSampleSize = tilt.sampleSize();
        float[] tiltSample = new float[tiltSampleSize];
        tilt.getAngleMode().fetchSample(tiltSample, 0);

        float startAngle = tiltSample[0];

        double startTime = System.currentTimeMillis();
        if (angle < 0) {
        	double rotatedAngle = tiltSample[0] - startAngle;
        	while (rotatedAngle > angle) {
	        	Motor.C.backward();
	        	Motor.B.forward();
	        	tilt.getAngleMode().fetchSample(tiltSample, 0);
	        	rotatedAngle = tiltSample[0] - startAngle;
	        	System.out.println(rotatedAngle + " " + angle);
	        	double currTime = System.currentTimeMillis();
	        	if (currTime - startTime > 5000) break;
        	}

        } else {
        	double rotatedAngle = tiltSample[0] - startAngle;
        	while (rotatedAngle < angle) {
	        	Motor.B.backward();
	        	Motor.C.forward();
	        	tilt.getAngleMode().fetchSample(tiltSample, 0);
	        	rotatedAngle = tiltSample[0] - startAngle;
	        	System.out.println(rotatedAngle + " " + angle);
	        	double currTime = System.currentTimeMillis();
	        	if (currTime - startTime > 5000) break;
        	}
        }
        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);

    }
    private static void steerToPizzaLocation(char pizzachoice) {
        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);

        Motor.B.rotate((int)(20 / l), true);
        Motor.C.rotate((int)(20 / l));
        if (pizzachoice == 0) { // left one
        	rotateRobot(90);
            //Motor.B.rotate(-RIGHT_ANGLE, true);
            //Motor.C.rotate(RIGHT_ANGLE);
        } else {
        	rotateRobot(-90);
        }
        Motor.B.rotate((int)((55-ROBOT_LENGTH) / l), true);
        // FIXME - should deduct by the length of robot
        Motor.C.rotate((int)((55-ROBOT_LENGTH) / l));
    }

    private static void fetchPizza() {
        Motor.A.setSpeed(BASE_SPEED);
        Motor.A.rotate(GRIP_ANGLE);

    }

    private static void steerBackToCenter(char pizzachoice) {
        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);
        Motor.B.rotate(-(int)((55-ROBOT_LENGTH) / l), true);
        Motor.C.rotate(-(int)((55-ROBOT_LENGTH) / l));

        if (pizzachoice == 0) { // left pizza. I should turn right now
        	rotateRobot(-90);

        } else {
        	rotateRobot(90);
        }


    }

    private static double distdiff(double x, double y, double x_tgt, double y_tgt) {
        return Math.sqrt((x-x_tgt)*(x-x_tgt) + (y-y_tgt)*(y-y_tgt));
    }

    private static void steerToColor(char[] userSelectionList) {
        // Steers to the color circle facing exactly north (theta=90 deg)
        // Needs to avoid obstacles and record distances
        x = 0; // in centimeters
        y = 2; // in centimeters
        double x_tgt, y_tgt;
        double bprev = 0, cprev = 0;



        int gyroSampleSize = tilt.sampleSize();

        float[] tiltsample = new float[gyroSampleSize];

        int colorSampleSize = color.sampleSize();
        float[] colorSample = new float[colorSampleSize];


        if (userSelectionList[1] == 0) {
            x_tgt = -29.5;
            y_tgt = 192.7 + 3;
        } else if (userSelectionList[1] == 1) {
            x_tgt = 0;
            y_tgt = 192.7;
        } else { // (userSelectionList[1] == 2)
            x_tgt = 30.5;
            y_tgt = 195.5;
        }
        while (distdiff(x, y, x_tgt, y_tgt) > 1.0) {
            double K_p = 5;
            double theta_d = 0;

            /*
            if (x_tgt > x) {
                theta_d = Math.atan((y_tgt - y) / (x_tgt - x)) * 180 / Math.PI;
            } else if (x_tgt < x){
                theta_d = Math.atan((y_tgt - y) / (x_tgt - x)) * 180 / Math.PI;
            } else {
                theta_d = 90;
            }
            */
            if (Math.abs(x_tgt - x) < 0.001) {
            	theta_d = 90;
            } else if (x_tgt > x) {
            	theta_d = Math.atan((y_tgt - y) / (x_tgt - x)) / Math.PI * 180;
            } else if (x_tgt < x) {
            	theta_d = Math.atan((y_tgt - y) / (x_tgt - x)) / Math.PI * 180 + 180;
            }
            tilt.getAngleMode().fetchSample(tiltsample, 0);
            double theta = tiltsample[0] - GYRO_OFFSET + 90;
            rotateRobot(theta_d - theta);

            Motor.B.forward();
            Motor.C.forward();

            // update x and y
            double bcurr = Motor.B.getPosition();
            double ccurr = Motor.C.getPosition();
            double brot = bcurr - bprev;
            double crot = ccurr - cprev;
            bprev = bcurr;
            cprev = ccurr;
            double vdt = (brot + crot) / 2 * l;
            x += (vdt * Math.cos(theta * Math.PI / 180));
            y += (vdt * Math.sin(theta * Math.PI / 180));

            // Add obstacle avoidance code

            color.getRedMode().fetchSample(colorSample, 0);
            double THRESHOLD = 1.0;
            if (colorSample[0] * 100 > THRESHOLD) {
                double[] stateRecorder = new double[3];
                stateRecorder[0] = x;
                stateRecorder[1] = y;
                stateRecorder[2] = theta;
                circumventObstacle(stateRecorder);
                x = stateRecorder[0];
                y = stateRecorder[1];
                theta = stateRecorder[2];
            }

            System.out.println(String.format("x=%.1f, y=%.1f, t=%.1f", x,y,theta));
        }

        // Now that you arrived at the circle. Rotate back to the north.
        tilt.fetchSample(tiltsample, 0);
        double theta = tiltsample[0] - GYRO_OFFSET;
        rotateRobot(90 - theta);


    }

    private static void circumventObstacle(double[] stateRecorder) {
        // Turn right 90 deg;

        rotateRobot(-90);



        int sonicSampleSize = sonic.sampleSize();
        float[] sonicsample = new float[sonicSampleSize];
        sonic.fetchSample(sonicsample, 0);

        double DIST_MAX = 0.5; // in meters. Same as sonic sampling
        double DIST_FOLLOW = 0.15;

        // Terminate when ultrasound sensor detects infinite distance
        while (sonicsample[0] < DIST_MAX) {

            // FIXME - make sure all sensor arrays are initialized before visited
            sonic.fetchSample(sonicsample, 0);

            double K_p = 0.05;
            double err = DIST_FOLLOW - sonicsample[0];
            int v_diff = (int)(err * K_p);

            double leftSpeed = capping(BASE_SPEED - v_diff);
            double rightSpeed = capping(BASE_SPEED + v_diff);
            Motor.B.setSpeed((int)leftSpeed));
            Motor.C.setSpeed((int)rightSpeed);

            Motor.B.forward();
            Motor.C.forward();
        }

        Motor.B.rotate((int)ROBOT_LENGTH);
        Motor.C.rotate((int)ROBOT_LENGTH);

    }

    private static int capping(int v) {
        return Math.max(0.5*BASE_SPEED, Math.min(1.5*BASE_SPEED, v));
    }

    private static void pivotAndDeliverPizza(char[] userSelectionList) {
        // Assume that the robot already arrived at either of green, blue, and red circle.
        if (userSelectionList[1] == 0) {
            // turn left for 28.5 deg
        	rotateRobot(28.5);
        } else if (userSelectionList[1] == 1) {
            ; // Go straight
        } else {
        	rotateRobot(-30.5);
        }


        // Then proceed to delivering the pizza. Since our ultrasonic sensor is at left side,
        // if the house is supposed to be at the right side, turn around and go backwards.
        boolean backwards = false;
        if (userSelectionList[2] == 1) { // house at the right side
            backwards = true;
            rotateRobot(180);
        }

        int houseCount = 0;
        boolean houseDetected = false;
        boolean houseDetectedPrev = false;
        double bprev = Motor.B.getPosition();
        double cprev = Motor.C.getPosition();

        double[] medianFilter = {0,0,0};

        double startTime = System.currentTimeMillis();
        while (y < 356.7) {
        	double currTime = System.currentTimeMillis();
            if (houseCount == userSelectionList[3] || (currTime - startTime) > 60000) {
                dropPizza();
                // let robot steer until reaching the base line.
                double diff_y = 356.7 - y;
                double alpha = 0;
                if (userSelectionList[1] == 0) {
                    alpha = 28.5;
                } else if (userSelectionList[1] == 2) {
                    alpha = 30.5;
                }
                alpha = alpha * Math.PI / 180;
                double comp_length = (diff_y) / Math.cos(alpha);
                Motor.B.rotate((int)(comp_length / l * (backwards ? (-1) : 1)), true);
                Motor.C.rotate((int)(comp_length / l * (backwards ? (-1) : 1)));

                Motor.B.stop();
                Motor.C.stop();

                if (backwards) {
                	rotateRobot(180);
                }
                break;
            }
            Motor.B.setSpeed(BASE_SPEED);
            Motor.C.setSpeed(BASE_SPEED);
            if (backwards) {
                Motor.B.backward();
                Motor.C.backward();
            } else {
                Motor.B.forward();
                Motor.C.forward();
            }
            double bcurr, ccurr;
            bcurr = Motor.B.getPosition();
            ccurr = Motor.C.getPosition();
            double brot = bcurr - bprev;
            double crot = ccurr - cprev;
            double vdt = (brot + crot) / 2 * l;

            int gyroSampleSize = tilt.sampleSize();
            float[] tiltSample = new float[gyroSampleSize];
            tilt.getAngleMode().fetchSample(tiltSample, 0);
            double theta = tiltSample[0] - GYRO_OFFSET;
            x += (vdt * Math.cos(theta * Math.PI / 180) * (backwards? (-1) : 1));
            y += (vdt * Math.sin(theta * Math.PI / 180) * (backwards? (-1) : 1));

            bprev = bcurr;
            cprev = ccurr;

            // Record distance. Switch the 'houseDetected' property when distance goes beyond the threshold.
            // To avoid effect of fluctuation, applied a median filter of size N=3 to it.
            // FIXME - I assume all sensor data (including those Infinity) are comparable
            int sonicSampleSize = sonic.sampleSize();
            float[] sonicsample = new float[sonicSampleSize];
            sonic.fetchSample(sonicsample, 0);
            medianFilter[0] = medianFilter[1];
            medianFilter[1] = medianFilter[2];
            medianFilter[2] = sonicsample[0];
            double processedDist = median(medianFilter);

            if (processedDist > 1.0) {
                houseDetectedPrev = houseDetected;
                houseDetected = false;
            } else if (processedDist < 0.4){
                houseDetectedPrev = houseDetected;
                houseDetected = true;
            }
            if ((!houseDetectedPrev) && houseDetected) { // @posedge of detection of house
                houseCount++;
            }

        }

    }

    private static void dropPizza() {
        // rotate left 90 deg, forward a bit, drop, then back a bit, rotate back

    	rotateRobot(90);

        Motor.B.rotate((int)(10 / l), true);
        Motor.C.rotate((int)(10 / l));

        Motor.A.setSpeed(BASE_SPEED);
        Motor.A.rotate(-GRIP_ANGLE);

        Motor.B.rotate(-(int)(10 / l), true);
        Motor.C.rotate(-(int)(10 / l));

        rotateRobot(-90);
    }

    private static double median(double[] A) {
        double[] B = Arrays.copyOf(A, A.length);
        Arrays.sort(B);
        if (A.length % 2 == 0) {
            return (B[B.length / 2] + B[B.length / 2 + 1]) / 2.0;
        } else {
            return B[(int)(B.length / 2)];
        }
    }
    private static void steerBackAfterDelivery() {
        // Now we know x, y, and theta
        int gyroSampleSize = tilt.sampleSize();
        float[] tiltSample = new float[gyroSampleSize];
        tilt.getAngleMode().fetchSample(tiltSample, 0);

        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);

        // Steer towards positive x axis orientation
        tiltSample[0] -= GYRO_OFFSET;
        rotateRobot(-(90-tiltSample[0]));

        // First steer to the closest position on boundary line at x=110, y=396.5
        double delta_x = 110 - x;
        Motor.B.rotate((int)(delta_x / l), true);
        Motor.C.rotate((int)(delta_x / l));

        // Then steer all the way back to the origin position and status
        rotateRobot(-90);

        Motor.B.rotate((int) (397 / l), true);
        Motor.C.rotate((int) (397 / l));

        rotateRobot(-90);

        Motor.B.rotate((int) (126.5 / l), true);
        Motor.C.rotate((int) (126.5 / l));

        rotateRobot(-90);

    }
    public static void main(String[] args) throws Exception {
        initialize();
        char[] userSelectionList = getInputInterface();

        steerToPizzaLocation(userSelectionList[0]);
        fetchPizza();
        steerBackToCenter(userSelectionList[0]);
        steerToColor(userSelectionList);

        //pivotAndDeliverPizza(userSelectionList);

        //steerBackAfterDelivery();
    }
}
