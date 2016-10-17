//import lejos.hardware.Button;
import lejos.hardware.motor.*;
import java.util.*;

public class MotorThread extends Thread {
    private String motorID;
    private final int MOTORSPEED = 360;
    private final int MAXGRIP = 90;
    private final int MAXPITCH = 90;
    private final int MAXYAW = 90;

    public MotorThread(String mID) {
        motorID = mID;
        Motor.A.setSpeed(MOTORSPEED);
    }

    // A grip B pitch C yaw
    public void run () {
        if (motorID.equals('A')) {
            Motor.A.rotate(MAXGRIP);
        } else if (motorID.equals('B')) {
            Motor.B.rotate(MAXPITCH);
        } else if (motorID.equals('C')){
            Motor.C.rotate(MAXYAW);
        }
    }
}
