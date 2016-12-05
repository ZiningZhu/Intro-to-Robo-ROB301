import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class RotationTest {
	private static EV3UltrasonicSensor sonic;
    private static EV3ColorSensor color;
    private static EV3GyroSensor tilt;
    private static final int BASE_SPEED = 120;
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
	        	System.out.println(rotatedAngle + " " + tiltSample[0]);
	        	
        	}

        } else {
        	double rotatedAngle = tiltSample[0] - startAngle;
        	while (rotatedAngle < angle) {
	        	Motor.B.backward();
	        	Motor.C.forward();
	        	tilt.getAngleMode().fetchSample(tiltSample, 0);
	        	rotatedAngle = tiltSample[0] - startAngle;
	        	System.out.println(rotatedAngle + " " + tiltSample[0]);
	        	
        	}
        }
        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);

    }
	public static void main(String[] args) {
		initialize();
		
		rotateRobot(90);

		
		Motor.B.rotate(200, true);
		Motor.C.rotate(200);
		Motor.B.rotate(-200, true);
		Motor.C.rotate(-200);
		rotateRobot(-90);
		rotateRobot(180);
		rotateRobot(-180);

	}

}
