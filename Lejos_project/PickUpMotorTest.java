import lejos.hardware.motor.*;

public class PickUpMotorTest {

    public static void main(String[] args) {
        Motor.A.setSpeed(100);
        Motor.A.rotate(-900); // Let it open completely

        Motor.A.rotate(100);
    }
}
