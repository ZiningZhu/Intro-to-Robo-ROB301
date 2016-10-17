import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3ColorSensor;

public class ObjDetect {
	private static final int[] LOC_YAW = 
			{0, 60, 120, 180, 240, 300};
	
	
	public static void main(String[] args) {
		
		EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S1);
		EV3ColorSensor light = new EV3ColorSensor(SensorPort.S3);
		// A grip B pitch C yaw
		Motor.A.setSpeed(200);
		Motor.B.setSpeed(200);
		Motor.C.setSpeed(200);
		

		for (int i = 0; i < 6; i++) {
			Motor.C.rotate(-150);
			Motor.B.rotate(LOC_YAW[i]);
			float[] sample = new float[touch.sampleSize()];
			touch.fetchSample(sample, 0);
			if (sample[0] == '1') { // Pressed
				System.out.println(String.format("Location %d Yes", i));
			} else {
				System.out.println(String.format("Location %d No", i));
			}
		}
	}

}
