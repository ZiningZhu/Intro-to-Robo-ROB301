import lejos.hardware.motor.*;
import lejos.hardware.Button;
public class CurvedPath {

	public static void main(String[] args) {
		
		while (!Button.ENTER.isDown()) {// Just block the Thread until btn is pressed
			int i=0;
			i += 1;
		}
		Motor.B.setSpeed(360);
		Motor.C.setSpeed(360);
		double l = 0.0463888;
		
		
		
		// Forward
		Motor.B.rotate((int)(181 / l), true);
		Motor.C.rotate((int)(181 / l));
		
		// Turn2
		Motor.B.setSpeed(224);
		Motor.B.rotate(1043, true);
		Motor.C.rotate(1675);
		
	}

}
