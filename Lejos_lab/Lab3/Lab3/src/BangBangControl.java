import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class BangBangControl {
	
	private static final int V0 = 200;
	private static final int DIFF = 100;
	public static void main(String[] args) {
		// B left C right
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);
		Motor.B.setSpeed(V0);
		Motor.C.setSpeed(V0);
		
		float target = (float)18.00; // TO be calibrated
		
		while (!Button.ENTER.isDown()) {
			int sampleSize = color.sampleSize();
			float[] sample = new float[sampleSize];
			color.getRedMode().fetchSample(sample, 0);
			
			float err = sample[0] * 100 - target;
			if (err < 0) {
				Motor.B.setSpeed(V0+DIFF);
				Motor.C.setSpeed(V0-DIFF);
				Motor.B.forward();
				Motor.C.forward();
				System.out.println(err);
			} else {
				Motor.B.setSpeed(V0-DIFF);
				Motor.C.setSpeed(V0+DIFF);
				Motor.B.forward();
				Motor.C.forward();
				System.out.println(err);
			}
			
			
		}

	}

}
