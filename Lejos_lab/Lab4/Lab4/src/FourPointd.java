import lejos.hardware.Button;
import lejos.hardware.motor.Motor;

public class FourPointd {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		while (!Button.ENTER.isDown()) {// Just block the Thread until btn is pressed
			int i=0;
			i += 1;
		}
		int SPEED = 200;
		double DISCOUNT = 1.00;
		int ANGLE = 210;
		Motor.B.setSpeed((int)(SPEED*DISCOUNT));
		Motor.C.setSpeed((int)(SPEED));
		double l = 0.0463888;
		
		
		// Forward
		Motor.B.rotate((int)(100 * DISCOUNT / l), true);
		Motor.C.rotate((int)(100 / l));
		
		// Turn, counter-clockwise
		Motor.B.rotate(-ANGLE, true);
		Motor.C.rotate(ANGLE);
				
		// Forward
		Motor.B.rotate((int)(100 * DISCOUNT / l), true);
		Motor.C.rotate((int)(100 / l));
		
		// Turn, counter-clockwise
		Motor.B.rotate(-ANGLE, true);
		Motor.C.rotate(ANGLE);
				
		// Forward
		Motor.B.rotate((int)(100 * DISCOUNT / l), true);
		Motor.C.rotate((int)(100 / l));
		
		// Turn, counter-clockwise
		Motor.B.rotate(-ANGLE, true);
		Motor.C.rotate(ANGLE);
				
		// Forward
		Motor.B.rotate((int)(98 * DISCOUNT / l), true);
		Motor.C.rotate((int)(98 / l));
		
		// Turn, counter-clockwise
		Motor.B.rotate(-ANGLE, true);
		Motor.C.rotate(ANGLE);
	}

}
