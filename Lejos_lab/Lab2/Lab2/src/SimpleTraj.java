import lejos.hardware.motor.*;
public class SimpleTraj {

	public static void main(String[] args) {
		// A grip B pitch C yaw
		Motor.A.setSpeed(360);
		
		Motor.B.setSpeed(360);
		Motor.C.setSpeed(360);
		
		int MAXPITCH = 300;
		Motor.B.rotate(-MAXPITCH); //rise
		
		int MAXYAW = 200;
		Motor.C.rotate(MAXYAW);
		Motor.C.rotate(-MAXYAW);
		Motor.C.rotate(-MAXYAW);
		Motor.C.rotate(MAXYAW);
		
		Motor.B.rotate(MAXPITCH); // down
	}

}
