import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.motor.*;
import java.util.*;

public class LessSimple {
	public static void main (String[] args) {
		// A grip B pitch C yaw
        /*
		MotorThread t1 = new MotorThread(Character.toString('B'));
        MotorThread t2 = new MotorThread(Character.toString('C'));
        System.out.println("");
        t1.run();
        t2.run();
        // Now Motor A and B runs at the same time.
         
        */
		Motor.B.setSpeed(300);
		Motor.C.setSpeed(400);
		Motor.B.rotate(-300, true); // non block
		Motor.C.rotate(400); // block the thread
		Motor.B.rotate(300, true);
		Motor.C.rotate(-400);
    }
}
