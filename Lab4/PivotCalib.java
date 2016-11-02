import lejos.hardware.motor.*;
public class PivotCalib {

	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub


		Motor.C.setSpeed(360);
		Motor.B.setSpeed(360);

        // Should make the car fully pivot (360 degrees)
        Motor.C.rotate(845, true);
        Motor.B.rotate(-845);

	}

}
