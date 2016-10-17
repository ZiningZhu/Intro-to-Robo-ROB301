import lejos.hardware.Button;
import lejos.hardware.motor.*;
public class DoubleMotor {

	private static int motorID;
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		while (!Button.ENTER.isDown()) {
			motorID = 0;
			MotorThread m1 = new MotorThread();
			
		}

	}

	
	
	public class MotorThread extends Thread {
		public void run() {
			if (motorID == 0) {
				Motor.A.setSpeed(360);
				Motor.A.rotate(90);
			} else {
				Motor.B.setSpeed(360);
				Motor.B.rotate(90);
			}
		}
		
		public void main (String args[]) {
			start();
		}
	} 
	

	

}
