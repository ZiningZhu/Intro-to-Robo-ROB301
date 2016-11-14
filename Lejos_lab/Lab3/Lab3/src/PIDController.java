import java.util.*;
import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
public class PIDController {
	private static final int V0 = 50;
	private static final int DIFF = 10;
	public static void main(String[] args) {
		// B left C right
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);
		Motor.B.setSpeed(V0);
		Motor.C.setSpeed(V0);
		
		float target = (float)18; // TO be calibrated
		
		float errint = (float)0.0;
		float[] errbuffer = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
		
		
		while (!Button.ENTER.isDown()) {
			int sampleSize = color.sampleSize();
			float[] sample = new float[sampleSize];
			color.getRedMode().fetchSample(sample, 0);
			
			float err = sample[0] * 100 - target;
			float Kp = (float)0.5;
			float Ki = (float)0.01;
			float Kd = (float)0.4;
			for (int i = 1; i < errbuffer.length; i++) {
				errbuffer[i-1] = errbuffer[i];
			}
			errbuffer[errbuffer.length-1] = err;
			errint = 0;
			for(int i = 0; i < errbuffer.length; i++) {
				errint += errbuffer[i];
			}
			
			float errdir = errbuffer[errbuffer.length-1] - errbuffer[errbuffer.length-2];
			
			float adj = err * Kp + errint * Ki + errdir * Kd;
			System.out.println(String.format("%.2f, %.2f", errdir, errint));
			Motor.B.setSpeed(V0 - DIFF * adj);
			Motor.C.setSpeed(V0 + DIFF * adj);
			
			Motor.B.forward();
			Motor.C.forward();
			
			
			
		}

	}

}
