import lejos.hardware.motor.*;
import lejos.hardware.Button;
public class Pos2pos {

	public static void main(String[] args) {
		
		while (!Button.ENTER.isDown()) {// Just block the Thread until btn is pressed
			int i=0;
			i += 1;
		}
		Motor.B.setSpeed(360);
		Motor.C.setSpeed(360);
		double l = 0.0463888;
		
		// Turn, counter-clockwise
		Motor.B.rotate(-29, true);
		Motor.C.rotate(29);
		
		// Forward
		Motor.B.rotate((int)(200 / l), true);
		Motor.C.rotate((int)(200 / l));
		
		// Turn2
		Motor.B.rotate(-298, true);
		Motor.C.rotate(298);
		
	}

}
