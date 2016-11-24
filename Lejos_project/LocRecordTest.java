import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import java.lang.*;
import java.util.*;

public class LocRecordTest {



    public static void main(String[] args) {
        EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S1);
        EV3GyroSensor tilt = new EV3GyroSensor(SensorPort.S3);
        int tiltSampleSize = tilt.sampleSize();
        float[] tiltsample = new float[tiltSampleSize];
        
        int BASE_SPEED = 240;
        Motor.B.setSpeed(BASE_SPEED);
        Motor.C.setSpeed(BASE_SPEED);

        int i=0;
        while (!Button.ENTER.isDown()) {
            i++;
        }
        while (Button.ENTER.isDown()) {
            i++;
        }

        // Record the step time
        ArrayList<Integer> stepTime = new ArrayList<Integer>();
        
        // Record to calculate how far motors rotate in each step
        ArrayList<Double> bAngles = new ArrayList<Double>();
        ArrayList<Double> cAngles = new ArrayList<Double>();
        double bprev_pos = 0;
        double cprev_pos = 0;
        Motor.B.resetTachoCount();
        Motor.C.resetTachoCount();
        
        double xpos = 0;
        double ypos = 0;
        
        double startTime = System.currentTimeMillis();
        double tgtDistance = 20;
        double dt = 0;
        double brot=0, crot=0;
        // TODO - Follow to go around obstacles; then display the current location, and go back.
        while (!Button.ENTER.isDown()) {
			double currTime = System.currentTimeMillis();
			double elapsedTime = currTime - startTime;

			int sonicSampleSize = sonic.sampleSize();
			float[] sonicsample = new float[sonicSampleSize];
			sonic.fetchSample(sonicsample, 0);
			LCD.clear();
			//System.out.println(sonicsample[0]*100);




			if (elapsedTime > 10000) break;

			tilt.getAngleMode().fetchSample(tiltsample, 0);
			double theta = tiltsample[0];
			xpos += (crot + brot) / 2 * Math.cos(theta);
			ypos += (crot + brot) / 2 * Math.sin(theta);
					
			double err = sonicsample[0] * 100 - tgtDistance;

			double Kp = 5;
			double DIFF = Math.max(-100, Math.min(100, Kp * err));
			int Bspeed = BASE_SPEED - (int)DIFF;
			int Cspeed = BASE_SPEED + (int)DIFF;
			Motor.B.setSpeed(Bspeed);
			Motor.C.setSpeed(Cspeed);

			Motor.C.forward();
			Motor.B.forward();

            double cpos = Motor.C.getPosition();
            double bpos = Motor.B.getPosition();
            
            brot = -0.0002 * (bpos - bprev_pos);
          
            crot = -0.0002 * (cpos - cprev_pos);
            
            
            bprev_pos = bpos;
            cprev_pos = cpos;
            

			double endingTime = System.currentTimeMillis();
			stepTime.add((int) (endingTime - currTime));
			dt = (int)(endingTime - currTime);
			
			
			System.out.println("(" + xpos + "," + ypos + ")");
			/*
			 if (bpos > 1000.00) {
			 
				break; // Corresponds to 50cm
			}*/
		}
        Motor.B.setSpeed(0);
        Motor.C.setSpeed(0);
        Motor.B.rotate(0, true);
        Motor.C.rotate(0);
        while(!Button.ENTER.isDown()) ;

    }
}
