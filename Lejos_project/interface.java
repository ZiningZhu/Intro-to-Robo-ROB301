import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import java.lang.*;
import java.util.*;

public class Interface {

    public static void PizzaLocation(int choice){
        if(choice == 1){
            System.out.println("Pizza Location:");
            System.out.println("-> Left");
            System.out.println("   Right");
        } else if(choice == 2){
            System.out.println("Pizza Location:");
            System.out.println("   Left");
            System.out.println("-> Right");
        }
    }

    public static void CircleLocation(int choice){
        if(choice == 1){
            System.out.println("Circle Location:");
            System.out.println("-> Left");
            System.out.println("   Center");
            System.out.println("   Right");
        } else if(choice == 2){
            System.out.println("Circle Location:");
            System.out.println("   Left");
            System.out.println("-> Center");
            System.out.println("   Right");
        } else if(choice == 3){
            System.out.println("Circle Location:");
            System.out.println("   Left");
            System.out.println("   Center");
            System.out.println("-> Right");
        }
    }
    public static void HouseSide(int choice){
        if(choice == 1){
            System.out.println("House Side:");
            System.out.println("-> Left");
            System.out.println("   Right");
        } else if(choice == 2){
            System.out.println("House Side:");
            System.out.println("   Left");
            System.out.println("-> Right");
        }
    }

    public static char[] Interface() {
        while (!Button.ENTER.isDown()) {
            int choice = 1;
            LCD.clear();
            PizzaLocation(choice);
            if(Button.UP.isDown() || Button.DOWN.isDown()){
                while (Button.UP.isDown() || Button.DOWN.isDown);
                if(choice == 2) choice = 1;
                else choice = 2;
            }
        }
        while(Button.ENTER.isDown());
        int pizzachoice = choice;


        while (!Button.ENTER.isDown()) {
            int choice = 1;
            LCD.clear();
            CircleLocation(choice);
            if(Button.UP.isDown()){
                while(Button.UP.isDown());
                if(choice == 3) choice = 2;
                else if (choice == 2) choice = 1;
                else choice = 3
            }
            if (Button.DOWN.isDown()){
                while(Button.DOWN.isDown());
                if(choice == 3) choice = 1;
                else if (choice == 2) choice = 3;
                else choice = 2
            }
        }
        while(Button.ENTER.isDown());
        int circlechoice = choice;

        while (!Button.ENTER.isDown()) {
            int choice = 1;
            LCD.clear();
            HouseSide(choice);
            if(Button.UP.isDown() || Button.DOWN.isDown()){
                while (Button.UP.isDown() || Button.DOWN.isDown);
                if(choice == 2) choice = 1;
                else choice = 2;
            }
        }
        while(Button.ENTER.isDown());
        char houseside = (choice == 1)? 'l':'r';

        while (!Button.ENTER.isDown()) {
            int choice = 1;
            LCD.clear();
            System.out.println("Number: ", choice);
            if(Button.UP.isDown() && choice != 1){
                while Button.UP.isDown();
                choice = choice - 1;
            }
            if(Button.DOWN.isDown()){
                while Button.DOWN.isDown();
                choice = choice + 1;
            }
        }
        while(Button.ENTER.isDown());
        int housenumber = choice;

        char[] list = [pizzachoice ,circlechoice ,houseside ,housenumber];
        return list;

    }
}
