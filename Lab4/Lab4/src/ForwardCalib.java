
import lejos.hardware.motor.*;
public class ForwardCalib {

	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub


		Motor.C.setSpeed(360);
		Motor.B.setSpeed(360);

        // Should make the car go forward for 17.28cm 
		// actual: 16.7cm
		
        Motor.C.rotate(360, true);
        Motor.B.rotate(360);

	}
}
