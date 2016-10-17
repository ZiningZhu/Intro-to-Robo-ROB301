import lejos.hardware.motor.*;
import lejos.hardware.Button;
public class ForwardTest {

	public static void main(String[] args) {
		// B left C right
		Motor.B.setSpeed(360);
		Motor.C.setSpeed(360);
		Motor.B.forward();
		Motor.C.forward();
	}

}
