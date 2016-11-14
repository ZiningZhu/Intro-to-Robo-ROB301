import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import java.lang.Math;

public class FollowWall {

	private static double lambda = 0.0463888;
	// lambda = pi * d / 360.
	// Usage: theta * lambda = x
	public static void main(String[] args) throws Exception {
		EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S3);
		
		int BASE_SPEED = 240;
		Motor.C.setSpeed(BASE_SPEED);
		Motor.B.setSpeed(BASE_SPEED);

		while (!Button.ENTER.isDown()) {// Just block the Thread until btn is pressed
			int i=0;
			i += 1;
		}
		double startTime = System.currentTimeMillis();
		LCD.clear();
		double tgtDistance = 20;
		while (!Button.ENTER.isDown()) {
			int sampleSize = sonic.sampleSize();
			float[] sonicsample = new float[sampleSize];
			sonic.fetchSample(sonicsample, 0);
			LCD.clear();
			//System.out.println(sonicsample[0]*100);

			
			
			double currTime = System.currentTimeMillis(); 
			double elapsedTime = currTime - startTime;
			if (elapsedTime > 28000) {
				break;
			} else if (elapsedTime > 19000 & tgtDistance >20) {
				tgtDistance -= 2;
			} else if (elapsedTime > 10000) {
				tgtDistance = 30;
			}
			double err = sonicsample[0] * 100 - tgtDistance;
			
			System.out.println(String.format("%.2f %.2f %d", tgtDistance, sonicsample[0]*100, (int)(elapsedTime/1000.0)));
			double Kp = 5;
			double DIFF = Math.max(-100, Math.min(100, Kp * err));
			Motor.B.setSpeed(BASE_SPEED + (int)DIFF);
			Motor.C.setSpeed(BASE_SPEED - (int)DIFF);
			Motor.C.rotate(30, true);
			Motor.B.rotate(30, true);
		}
		sonic.close();

	}

}
