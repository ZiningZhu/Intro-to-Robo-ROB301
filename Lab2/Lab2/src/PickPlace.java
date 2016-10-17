import lejos.hardware.Button;
import lejos.hardware.motor.*;
import lejos.hardware.motor.*;

public class PickPlace {
    private final static int MAXSPEED = 360;
    private final static int GRIP = 120;
    private final static int PITCH_DOWN = 300;
    private final static int PITCH_UP = -300;
    private final static int YAW = 1560;
    public static void main(String[] args) {
    	
    	// A grip B pitch C yaw
        Motor.A.setSpeed(MAXSPEED);
        Motor.B.setSpeed(300);
        Motor.C.setSpeed(300);

        Motor.B.rotate(PITCH_UP);
        
        // Go to pos 1
        Motor.C.rotate(-2 * YAW);
        
        //Pick
        Motor.B.rotate(PITCH_DOWN);
        Motor.A.rotate(GRIP);
        Motor.B.rotate(PITCH_UP);
        
        // Go to pos 3
        Motor.C.rotate(2 * YAW);
        
        // Place
        Motor.B.rotate(PITCH_DOWN);
        Motor.A.rotate(-GRIP);
        Motor.B.rotate(PITCH_UP);
        
        // Go to pos 2
        Motor.C.rotate(-YAW);
        Motor.B.rotate(PITCH_DOWN); 
        Motor.A.rotate(GRIP);
        Motor.B.rotate(PITCH_UP);
        
        // Go to pos 3 and drop
        Motor.C.rotate(YAW);
        Motor.B.rotate((int)0.5 * PITCH_DOWN);
        Motor.A.rotate(-GRIP);
        

        // Reset loc
        Motor.B.rotate((int)0.5 * PITCH_UP);
        Motor.C.rotate(2 * YAW);
        Motor.B.rotate((int)PITCH_DOWN);

    }
}
