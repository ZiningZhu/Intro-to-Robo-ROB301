import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import java.lang.Math;
import java.util.ArrayList;
import java.util.*;

public class PizzaDelivery {
    private static final int BASE_SPEED = 360;
    // Base speed for motors.

    private static final double l = 0.0463888;
    // motor rotation factor. (length in cm) / l = motor rotation angle
    private static final int RIGHT_ANGLE = 210;
    // both motor shall rotate this angle in opposite direction at the same time
    // to achieve a right angle turn of the robot
    private static final int ROBOT_LENGTH = 10;
    // in centimeters
    private static final int GRIP_ANGLE = 120;
    // Angle the motor has to rotate to close the grip

    private static double x = 0;
    private static double y = 0;
    // Coordinates of the robot, at any given time point. In centimeters
    // To be updated in steerToColor() and pivotAndDeliverPizza() using a modified unicycle model.

    private static EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S1);
    private static EV3ColorSensor color = new EV3ColorSensor(SensorPort.S2);
    private static EV3GyroSensor tilt = new EV3GyroSensor(SensorPort.S3);
    // Three sensors, corresponding to our three sensor ports

    private static char[] getInputInterface() {
        char pizzachoice, circlechoice, houseside, housenumber;
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
        System.out.println("Pizza selected" + (pizzachoice==0?"left":"right")));

        LCD.clear();
        System.out.println("Select circle: LEFT/UP/RIGHT");
        while ((!Button.LEFT.isDown()) && (!Button.UP.isDown()) && (!Button.RIGHT.isDown())) ;
        String selectedCircleName = "";
        if (Button.LEFT.isDown()) {
            circlechoice = 0;
            selectedCircleName = "Green";
        } else if (Button.UP.isDown()) {
            circleChoice = 1;
            selectedCircleName = "Blue";
        } else {
            circleChoice = 2;
            selectedCircleName = "Red";
        }
        while ((Button.LEFT.isDown()) || (Button.UP.isDown()) || (Button.RIGHT.isDown())) ;
        System.out.println("Circle selected: " + selectedCircleName);

        LCD.clear();
        System.out.println("Select house side: LEFT/RIGHT");
        while ((!Button.LEFT.isDown()) && (!Button.RIGHT.isDown())) ;
        if (Button.LEFT.isDown()) houseside = 0;
        else houseside = 1;
        while ((Button.LEFT.isDown()) || (Button.RIGHT.isDown())) ;
        System.out.println("House side selected" + (houseside==0?"left":"right")));

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
                houseNumber += 1;
                System.out.println("Current number: " + housenumber);
                while (Button.UP.isDown()) ;
            }
            if (Button.DOWN.isDown()) {
                houseNumber -= 1;
                System.out.println("Current number: " + housenumber);
                while (Button.DOWN.isDown()) ;
            }
        }
        System.out.println("House number confirmed: " + housenumber);

        char[] res = new char[4];
        res[0] = pizzachoice;
        res[1] = circlechoice;
        res[2] = houseside;
        res[3] = housenumber;
        return res;
    }


    private static void steerToPizzaLocation(pizzachoice) {
        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);

        Motor.B.rotate((int)(12 / l), true);
        Motor.C.rotate((int)(12 / l));
        if (pizzachoice == 0) { // left one
            Motor.B.rotate(-RIGHT_ANGLE, true);
            Motor.C.rotate(RIGHT_ANGLE);
        } else {
            Motor.B.rotate(RIGHT_ANGLE, true);
            Motor.C.rotate(-RIGHT_ANGLE);
        }
        Motor.B.rotate((int)((575-ROBOT_LENGTH) / l), true);
        // FIXME - should deduct by the length of robot
        Motor.C.rotate((int)((575-ROBOT_LENGTH) / l));
    }

    private static void fetchPizza() {
        Motor.A.setSpeed(BASE_SPEED);
        Motor.A.rotate(GRIP_ANGLE);

    }

    private static void steerBackToCenter(pizzachoice) {
        Motor.B,setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);
        Motor.B.rotate(-(int)((57.5-ROBOT_LENGTH) / l), true);
        Motor.C.rotate(-(int)((57.5-ROBOT_LENGTH) / l));

        if (pizzachoice == 0) { // left pizza. I should turn right now
            Motor.B.rotate(RIGHT_ANGLE, true);
            Motor.C.rotate(-RIGHT_ANGLE);
        } else {
            Motor.B.rotate(-RIGHT_ANGLE, true);
            Motor.C.rotate(RIGHT_ANGLE);
        }


    }

    private static void distdiff(double x, double y, double x_tgt, double y_tgt) {
        return Math.sqrt((x-x_tgt)*(x-x_tgt) + (y-y_tgt)*(y-y_tgt));
    }

    private static void steerToColor(userSelectionList) {
        // Needs to avoid obstacles and record distances
        x = 0; // in centimeters
        y = 12; // in centimeters
        double x_tgt, y_tgt;
        double bprev = 0, cprev = 0;



        int gyroSampleSize = tilt.sampleSize();
        float[] tiltSample = new float[gyroSampleSize];

        int colorSampleSize = color.sampleSize();
        float[] colorSample = new float[colorSampleSize];


        if (userSelectionList[1] == 0) {
            x_tgt = -29.5;
            y_tgt = 1927 + 30;
        } else if (userSelectionList[1] == 1) {
            x_tgt = 0;
            y_tgt = 1927;
        } else if (userSelectionList[1] == 2) {
            x_tgt = 305;
            y_tgt = 1955;
        }
        while (distdiff(x, y, x_tgt, y_tgt) > 10) {
            double K_p = 5;
            double theta_d = 0;
            if (x_tgt > x) {
                theta_d = Math.atan((y_tgt - y) / (x_tgt - x));
            } else if (x_tgt < x){
                theta_d = Math.atan((y_tgt - y) / (x_tgt - x));
            } else {
                theta_d = Math.pi / 2;
            }
            tilt.getAngleMode().fetchSample(tiltsample, 0);
            double theta = tiltsample[0] + Math.pi / 2;
            double v_diff = K_p * (theta_d - theta);
            Motor.B.setSpeed(BASE_SPEED - v_diff);
            Motor.C.setSpeed(BASE_SPEED + v_diff);
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
            x += (vdt * Math.cos(theta));
            y += (vdt * Math.sin(theta));

            // Add obstacle avoidance code
            color.getRedMode().fetchSample(sample, 0);
            if (colorSample[0] > THRESHOLD) {
                double[] stateRecorder = new double[3];
                stateRecorder[0] = x;
                stateRecorder[1] = y;
                stateRecorder[2] = theta;
                circumventObstacle(stateRecorder);
                x = stateRecorder[0];
                y = stateRecorder[1];
                z = stateRecorder[2];
            }


        }


    }

    private static void circumventObstacle(double[] stateRecorder) {
        // Turn right 90 deg;
        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);
        Motor.B.rotate(RIGHT_ANGLE, true);
        Motor.C.rotate(-RIGHT_ANGLE);



        int sonicSampleSize = sonic.sampleSize();
        float[] sonicsample = new float[sonicSampleSize];
        sonic.fetchSample(sonicsample, 0);

        double DIST_MAX = 0.5; // in meters. Same as sonic sampling
        double DIST_FOLLOW = 0.15;

        // Terminate when ultrasound sensor detects infinite distance
        while (sonicsample[0] < DIST_MAX) {

            // FIXME - make sure all sensor arrays are initialized before visited
            sonic.fetchSample(sonicsample, 0);

            double K_p = 0.1;
            double err = DIST_FOLLOW - sonicsample[0];
            int v_diff = (int)(err * K_p);

            Motor.B.setSpeed(BASE_SPEED - v_diff);
            Motor.C.setSpeed(BASE_SPEED + v_diff);

            Motor.B.forward();
            Motor.C.forward();
        }

    }
    private static void pivotAndDeliverPizza(userSelectionList) {
        // Assume that the robot already arrived at either of green, blue, and red circle.
        if (userSelectionList[1] == 0) {
            // turn left for 28.5 deg
            double angle = 28.5 / 90.0 * RIGHT_ANGLE;
            Motor.B.setSpeed(BASE_SPEED);
            Motor.C.setSpeed(BASE_SPEED);
            Motor.B.rotate(-(int)angle, true);
            Motor.C.rotate((int)angle);
        } else if (userSelectionList[1] == 1) {
            ; // Go straight
        } else {
            double angle = 30.5 / 90.0 * RIGHT_ANGLE;
            Motor.B.setSpeed(BASE_SPEED);
            Motor.C.setSpeed(BASE_SPEED);
            Motor.B.rotate((int)angle, true);
            Motor.C.rotate(-(int)angle);
        }


        // Then proceed to delivering the pizza. Since our ultrasonic sensor is at left side,
        // if the house is supposed to be at the right side, turn around and go backwards.
        int backwards = 0;
        if (userSelectionList[2] == 1) { // house at the right side
            backwards = 1;
            Motor.B.setSpeed(BASE_SPEED);
            Motor.C.setSpeed(BASE_SPEED);
            Motor.B.rotate(2*RIGHT_ANGLE, true);
            Motor.C.rotate(-2*RIGHT_ANGLE);
        }

        int houseCount = 0;
        boolean houseDetected = false;
        boolean houseDetectedPrev = false;
        double bprev = Motor.B.getPosition();
        double cprev = Motor.C.getPosition();

        double[] medianFilter = {0,0,0};

        while (y < 356.7) {
            if (houseCount == userSelectionList[3]) {
                dropPizza();
                // TODO - let robot steer until reaching the base line.
                double diff_y = 356.7 - y;
                double alpha = 0;
                if (userSelectionList[1] == 0) {
                    alpha = 28.5;
                } else if (userSelectionList[1] == 2) {
                    alpha = 30.5;
                }
                double comp_length = (diff_y) / Math.cos(alpha);
                Motor.B.rotate((int)(comp_length / l * (backwards ? (-1) : 1)), true);
                Motor.C.rotate((int)(comp_length / l * (backwards ? (-1) : 1)));

                Motor.B.stop();
                Motor.C.stop();

                if (backwards) {
                    Motor.B.rotate(2*RIGHT_ANGLE, true);
                    Motor.C.rotate(-2*RIGHT_ANGLE);
                }
                break;
            }
            Motor.B.setSpeed(BASE_SPEED);
            Motor.C.setSpeed(BASE_SPEED);
            if (backwards) {
                Motor.B.backwards();
                Motor.C.backwards();
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
            tilt.getAngleMode().fetchSample(tiltSample);
            double theta = tiltSample[0];
            x += (vdt * Math.cos(theta) * (backwards? (-1) : 1));
            y += (vdt * Math.sin(theta) * (backwards? (-1) : 1));

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
            medianFilter[2] = sonicSample[0];
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

        Motor.B.rotate(-RIGHT_ANGLE, true);
        Motor.C.rotate(RIGHT_ANGLE);

        Motor.B.rotate((int)(10 / l), true);
        Motor.C.rotate((int)(10 / l));

        Motor.A.setSpeed(BASE_SPEED);
        Motor.A.rotate(-GRIP_ANGLE);

        Motor.B.rotate(-(int)(10 / l), true);
        Motor.C.rotate(-(int)(10 / l));

        Motor.B.rotate(RIGHT_ANGLE);
        Motor.C.rotate(-RIGHT_ANGLE, true);
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
        tilt.getAngleMode().fetchSample(tiltSample);

        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);

        // Steer towards positive x axis orientation
        Motor.B.rotate(+(int)((Math.pi/2-tileSample[0]) / (Math.pi/2) * RIGHT_ANGLE), true);
        Motor.C.rotate(-(int)((Math.pi/2-tileSample[0]) / (Math.pi/2) * RIGHT_ANGLE));

        // First steer to the closest position on boundary line at x=110, y=396.5
        double delta_x = 110 - x;
        Motor.B.rotate((int)(delta_x / l), true);
        Motor.C.rotate((int)(delta_x / l));

        // Then steer all the way back to the origin position and status
        Motor.B.rotate(RIGHT_ANGLE, true);
        Motor.C.rotate(-RIGHT_ANGLE);

        Motor.B.rotate((int) (397 / l), true);
        Motor.C.rotate((int) (397 / l));

        Motor.B.rotate(RIGHT_ANGLE, true);
        Motor.C.rotate(-RIGHT_ANGLE);

        Motor.B.rotate((int) (126.5 / l), true);
        Motor.C.rotate((int) (126.5 / l));

        Motor.B.rotate(RIGHT_ANGLE, true);
        Motor.C.rotate(-RIGHT_ANGLE);

    }
    public static void main(String[] args) throws Exception {
        char[] userSelectionList = getInputInterface();

        steerToPizzaLocation(userSelectionList[0]);
        fetchPizza();
        steerBackToCenter(userSelectionList[0]);
        steerToColor(userSelectionList);

        pivotAndDeliverPizza(userSelectionList);

        steerBackAfterDelivery();
    }
}
