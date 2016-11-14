import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import java.lang.Math;
import java.util.ArrayList;
import java.util.*;



public class TeachingTrial {

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
		
		ArrayList<Integer> BSpeeds = new ArrayList<Integer>();
		ArrayList<Integer> CSpeeds = new ArrayList<Integer>();
		ArrayList<Integer> stepTime = new ArrayList<Integer>();
		
		while (!Button.ENTER.isDown()) {
			double currTime = System.currentTimeMillis(); 
			double elapsedTime = currTime - startTime;
			
			int sampleSize = sonic.sampleSize();
			float[] sonicsample = new float[sampleSize];
			sonic.fetchSample(sonicsample, 0);
			LCD.clear();
			//System.out.println(sonicsample[0]*100);

			
			

			if (elapsedTime > 30000) break;
			
			
			double err = sonicsample[0] * 100 - tgtDistance;
			
			System.out.println(String.format("%.2f %.2f %d", tgtDistance, sonicsample[0]*100, (int)(elapsedTime/1000.0)));
			double Kp = 5;
			double DIFF = Math.max(-100, Math.min(100, Kp * err));
			int Bspeed = BASE_SPEED + (int)DIFF;
			int Cspeed = BASE_SPEED - (int)DIFF;
			Motor.B.setSpeed(Bspeed);
			Motor.C.setSpeed(Cspeed);
			
			Motor.C.rotate(30, true);
			Motor.B.rotate(30, true);
			
			BSpeeds.add(Bspeed);
			CSpeeds.add(Cspeed);
			
			double endingTime = System.currentTimeMillis();
			stepTime.add((int) (endingTime - currTime));
		}
		
		LCD.clear();

		while (!Button.ENTER.isDown()) {// Just block the Thread until btn is pressed
			int i=0;
			i += 1;
		}
		
		
		
		int i = 0;
		int accumulated_short = 0;
		while (true) {
			double stTime = System.currentTimeMillis();
			if (i == BSpeeds.size()) break;
			//System.out.println(String.format("%ld %ld", BSpeeds.get(i)));
			Motor.B.setSpeed(BSpeeds.get(i));
			Motor.C.setSpeed(CSpeeds.get(i));
			
			Motor.B.rotate(30, true);
			Motor.C.rotate(30, true);
			i++;
			
			double currTime = System.currentTimeMillis();
			double sleepTime = stepTime.get(i-1) - currTime + stTime - 3;
			System.out.println(String.format("%.0f", sleepTime));
			if (sleepTime < 0) {
				accumulated_short -= sleepTime;
							
			}
			if (accumulated_short > sleepTime) {
				sleepTime = 0;
				accumulated_short -= sleepTime;
			} else {
				sleepTime -= accumulated_short;
				accumulated_short = 0;
			}
			Thread.sleep((long)(sleepTime));
		}
		
		sonic.close();
		

		
		
		
		

	}

}
