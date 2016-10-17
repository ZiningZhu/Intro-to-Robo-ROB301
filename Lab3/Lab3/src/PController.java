import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
public class PController {
	private static final int V0 = 80;
	private static final int DIFF = 30;
	public static void main(String[] args) {
		// B left C right
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);
		Motor.B.setSpeed(V0);
		Motor.C.setSpeed(V0);
		
		float target = (float)18; // TO be calibrated
		
		float preverr = (float)0.0;
		
		while (!Button.ENTER.isDown()) {
			int sampleSize = color.sampleSize();
			float[] sample = new float[sampleSize];
			color.getRedMode().fetchSample(sample, 0);
			
			float err = sample[0] * 100 - target;
			float Kp = (float)0.5;
			float adj = err * Kp;
			System.out.println(err);
			Motor.B.setSpeed(V0 - DIFF * adj);
			Motor.C.setSpeed(V0 + DIFF * adj);
			
			Motor.B.forward();
			Motor.C.forward();
			
			preverr = err;
		}

	}

}
