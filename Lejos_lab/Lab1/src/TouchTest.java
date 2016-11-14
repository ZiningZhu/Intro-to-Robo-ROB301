import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.motor.*;
public class TouchTest {
	private static boolean clockwise;
	public static void main(String[] args) {
		clockwise = true;
		EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S1);
		
		while (!Button.ENTER.isDown()) {
			float[] sample = new float[touch.sampleSize()];
			touch.fetchSample(sample, 0);
			if (sample[0] == '1'){
				System.out.println(sample[0]);
				System.out.println(touch.sampleSize());
				while (sample[0] == '1');
				clockwise = !clockwise;
			}
			Motor.A.setSpeed(360);
			if (clockwise) {
				Motor.A.rotate(45);
			} else {
				Motor.A.rotate(-45);
			}
			
			LCD.clear();
		}
		
		
		touch.close();
	}
}