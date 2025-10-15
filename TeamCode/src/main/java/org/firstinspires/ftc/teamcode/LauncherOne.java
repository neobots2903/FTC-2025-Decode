/*
* This class manages the robots launching system.
* Here we control dilvery of the balls to places in the game.
* */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * This class manages the robots launching system.
 * Here we control dilvery of the balls to places in the game.
 * */
public class LauncherOne {

    //This is an instance of the opMode of the robot.
    //Allows us to access the hardware configuration
    //and set up motors, sensors, etc.
    //Allows for communication to the telemetry system.
    OpMode opMode;

    //This color sensor is used in the launcher
    //to detect if a ball is present, or what
    //color it is.
    ColorSensor launcherColorSensor;

    //These are motors that launch the balls.
    //launchMotorTwo is spinning in reverse since both
    //these motors are on the same shaft.
    DcMotor launchMotorOne;
    DcMotor launchMotorTwo;

    //Constructor
    public LauncherOne(OpMode opMode) {

        //Take the instance of opMode and store
        //it in "opMode" being passed from the robot
        //class or wherever this class is crated as
        //an instance.
        this.opMode = opMode;

        //Intialize the color sensors
        initColorSensors();

        //Initalizing the Motors
        initMotors();

    }


    private void initMotors() {
        launchMotorOne = opMode.hardwareMap.get(DcMotor.class, "launchMotorOne");
        launchMotorTwo = opMode.hardwareMap.get(DcMotor.class, "launchMotorTwo");

        //Reverse the launchMotorTwo, since launchMotorTwo
        //and launchMotorOne are both on the same shaft, with opposite
        //input directions, this should make both motors spin the shaft
        //in the same direction.
        launchMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    //Sets up the color sensors
    //we are using in the launcher system
    private void initColorSensors() {

        //Intialize the launcher color sensor from
        //the hardware map.
        launcherColorSensor = opMode.hardwareMap.get(ColorSensor.class, "launcherColorSensor");
    }


    //Spin the launcher Motors to shoot the ball
    //Spins the motors indefiently until we call
    //the stop function. "stopLauncher()"
    //----
    //ARGUMENTS:
    //-"power" -> Runs the launch motors at this power
    public void runLauncher(double power) {
        launchMotorOne.setPower(power);
        launchMotorTwo.setPower(power);
    }


    //Kills the launcher, stops all motors
    //at 0.0 power.
    //launcherMotorOne and launcherMotorTwo
    //will stop (0 power)
    public void killLauncher() {
        launchMotorOne.setPower(0.0);
        launchMotorTwo.setPower(0.0);
    }


    //Returns the total color value detected by each
    //RGB value in the color sensor based on which
    //color you request in arg 1.
    //
    //ARGUMENTS:
    //
    //color -> The color value you want returned
    //"RED" = red value
    //"GREEN" = green value
    //"BLUE" = blue value
    //"ALPHA" = color opacity value
    public int launcherColorSensor_getColor(String color) {

        //Will hold the value we want to return, as requested by
        //first argument
        int value = 0;

        //Based on the value the 1st argument
        //wants to return, we set this value
        //from that color to the value.
        //If "color" is not a regonized color
        //or value, we return -1.
        //If red is requested
        if (color == "RED") {
            value = launcherColorSensor.red();
            //If green is requested
        } else if (color == "GREEN") {
            value = launcherColorSensor.green();
            //If blue is requested
        } else if (color == "BLUE") {
            value = launcherColorSensor.blue();
            //If alpha is requested
        } else if (color == "ALPHA") {
            value = launcherColorSensor.alpha();
            //If the keyword isn't regonized, return -1.
        } else {
            value = -1;
        }

        return value;
    }
}
