import lejos.hardware.motor.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import java.lang.Math;
import java.util.ArrayList;
import java.util.*;

public class PizzaDelivery {

    private double lambda_wheel = 1.0; // TODO

    private static void turnLeft(int theta, Motor mleft, Motor mright) {
        mleft.setSpeed(100);
        mright.setSpeed(-100);
        mleft.rotate((int)(theta * lambda_wheel));
        mright.rotate((int)(theta * lambda_wheel));
    }

    private static void turnRight(int theta) {

    }

    private static void clampHand(Motor motor) {

    }
    public static void main(String[] args) throws Exception {

        double pizzaLocX = 0.0;
        double pizzaLocY = 0.5;
        while (!Button.LEFT.isDown() && !Button.RIGHT.isDown()) {
            ;
        }

        if (Button.LEFT.isDown()) {
            pizzaLocX = -3.0;
        } else {
            pizzaLocX = 3.0;
        }

        Motor.A.setSpeed(10);
        Motor.B.setSpeed(100);
        Motor.C.setSpeed(100);

        turnLeft(90);
    }
}
