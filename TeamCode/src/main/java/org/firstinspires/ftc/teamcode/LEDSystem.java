package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
public class LEDSystem {


    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private OpMode opMode;



    //Constructor
    public LEDSystem(OpMode opMode) {
        this.opMode = opMode;

        initLEDs();
    }


    //Intializes all LEDs with the hardware
    //map so we can use them, runs setups if needed
    private void initLEDs() {

        //Setup the LED for indexer indication
        blinkinLedDriver = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void setIndicatorColor(String color) {

        if (color == "PURPLE") {
            //Logic to make light purple
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else if (color == "GREEN") {
            //Logic to make light green
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            //Logic to make light off
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

    }

    // Creates a party RGB display
    public void toggleShowMode(String color) {
        if (color == "PARTY") {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
        } else {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

    }

}
