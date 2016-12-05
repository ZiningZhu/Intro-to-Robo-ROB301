import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class PickPizzaAndReturn {
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
        /* Interface for gathering user selection.
        * pizza location, circle color, house side and house number are selected
        * and stored to a char[] array.
        * pizza location - 0 left; 1 right;
        * circle color - 0 green (left); 1 blue (middle); 2 red (right);
        * house side - 0 left; 1 right;
        * house number - integer ranging from 0 to 3, inclusive.
        */
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
        System.out.println("Init number: " + housenumber);
        boolean confirmed = false;
        while (!confirmed) {
            if (Button.ENTER.isDown()) {
                confirmed = true;
                while (Button.ENTER.isDown()) ;
                break;
            }
            if (Button.UP.isDown()) {
                housenumber += 1;
                if (housenumber > 4)
                    housenumber--;
                System.out.println("Curr number: " + housenumber);
                while (Button.UP.isDown()) ;
            }
            if (Button.DOWN.isDown()) {
                housenumber -= 1;
                if (housenumber < 0)
                    housenumber++;
                System.out.println("Curr number: " + housenumber);
                while (Button.DOWN.isDown()) ;
            }
        }
        System.out.println("Housenumber confirmed: " + housenumber);
        System.out.println("Press ENTER to run!");
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
    	/* angle<0 then turn right;
    	* angle>0 then turn left;
        * rotate until the robot rotates the desired angle (double)
        */
    	//angle = angle * 0.95;
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

    private static void goPickPizzaAndReturn(char pizzachoice) {
    	// steer to pizza location
    	Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);

        Motor.B.rotate((int)(20 / l), true);
        Motor.C.rotate((int)(20 / l));
        if (pizzachoice == 0) { // left one
        	rotateRobot(90);

        } else { // pizza is at right
        	rotateRobot(-90);
        }
        Motor.B.rotate((int)((55-ROBOT_LENGTH) / l), true);
        // should deduct by the length of robot
        Motor.C.rotate((int)((55-ROBOT_LENGTH) / l));
        
        
        // Fetch pizza
        Motor.A.setSpeed(BASE_SPEED);
        Motor.A.rotate(GRIP_ANGLE);
        
        
        // Steer Back to Center
        Motor.B.rotate(-(int)((55-ROBOT_LENGTH) / l), true);
        Motor.C.rotate(-(int)((55-ROBOT_LENGTH) / l));

        if (pizzachoice == 0) { // left pizza. Robot should turn right now
        	rotateRobot(-90);

        } else {
        	rotateRobot(90);
        }

        Motor.B.stop();
        Motor.C.stop();
        
        while (!Button.ENTER.isDown()) {
        	while (Button.ENTER.isDown()) ;
        }
        
    }
	public static void main(String[] args) {
		initialize();
	    char[] userSelectionList = getInputInterface();

	    goPickPizzaAndReturn(userSelectionList[0]);
	  

	}
}
