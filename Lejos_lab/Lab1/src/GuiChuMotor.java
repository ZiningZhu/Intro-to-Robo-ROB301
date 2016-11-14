import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;

public class GuiChuMotor {
	private static int pressed;
	public static void main(String[] args) {
		float i;
		int j = 20;
		pressed = 0;
		EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S1);
		LCD.clear();
		while (!Button.ENTER.isDown()) {
			Motor.A.setSpeed(j);
			int sampleSize = touch.sampleSize();
			float[] touchsample = new float[sampleSize];
			touch.fetchSample(touchsample, 0);
			LCD.clear();
			i = touchsample[0];
			if (touchsample[0] != pressed) {
				pressed = (int)touchsample[0];
				if (pressed == 0) {
					Motor.A.rotate(90);
					j+=20;
				} else {
					Motor.A.rotate(-90);
					j+=20;
				}
			}
			System.out.println(i);
			}
		touch.close();
		}

}
