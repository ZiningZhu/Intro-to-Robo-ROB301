import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
public class ColorSensorCal {
	
	public static void main(String[] args) {
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);
		LCD.clear();
		while (!Button.ENTER.isDown()) {
			int sampleSize = color.sampleSize();
			float[] redsample = new float[sampleSize];
			//color.getAmbientMode().fetchSample(ambsample, 0);
			
			color.getRedMode().fetchSample(redsample,  0);
			LCD.clear();
			/*
			float sum = 0;
			for (int i = 0; i < sampleSize; i++) {
				sum += redsample[i];
			}
			sum /= sampleSize;
			*/
			System.out.println(String.format("%d %.2f", sampleSize, redsample[0] * 100));
		}
		color.close();
	}
	
	
	
}
