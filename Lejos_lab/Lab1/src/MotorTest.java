import lejos.hardware.motor.*;
public class MotorTest {

	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		
		
		Motor.A.setSpeed(360);
		Motor.A.rotate(720);
		Motor.A.rotate(-720);
		Motor.B.setSpeed(360);
		Motor.B.rotate(-720);
		Motor.B.rotate(720);
	}

}
