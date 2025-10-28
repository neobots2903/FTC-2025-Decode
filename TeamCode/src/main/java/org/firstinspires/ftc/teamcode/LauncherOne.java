/*
* This class manages the robots launching system.
* Here we control dilvery of the balls to places in the game.
* */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This class manages the robots launching system.
 * Here we control dilvery of the balls to places in the game.
 * */
public class LauncherOne {

    //This is an instance of the opMode of the robot.
    //Allows us to access the hardware configuration
    //and set up motors, sensors, etc.
    //Allows for communication to the telemetry system.
    private OpMode opMode;

    //This color sensor is used in the launcher
    //to detect if a ball is present, or what
    //color it is.
    //private ColorSensor launcherColorSensor;

    //These are motors that launch the balls.
    //launchMotorTwo is spinning in reverse since both
    //these motors are on the same shaft.
    private DcMotor launchMotorOne;
    private DcMotor launchMotorTwo;

    //The servo for dropping the ball/object into the launcher!
    private CRServo shooterInput;
    double shooterInputOpenPosition = 1.0; //Position for the shooter input to be open.
    double shooterInputClosedPosition = 0.0; //Position for the shooter input to be closed.



    //Constructor
    public LauncherOne(OpMode opMode) {

        //Take the instance of opMode and store
        //it in "opMode" being passed from the robot
        //class or wherever this class is crated as
        //an instance.
        this.opMode = opMode;

        //Intialize the color sensors
        //initColorSensors();

        //Initalizing the Motors
        initMotors();

        //Intialize all servos onboard the robot
        initServos();
    }


    //Intializes the servos inside the robot.
    //Called from the constructor of this class.
    private void initServos() {

        //Create an instance of the shooter input servo in
        //the opmode hardware map.
        shooterInput = opMode.hardwareMap.get(CRServo.class, "shooterInput");
    }


    private void initMotors() {
        launchMotorOne = opMode.hardwareMap.get(DcMotor.class, "launchMotorOne");
        launchMotorTwo = opMode.hardwareMap.get(DcMotor.class, "launchMotorTwo");

        //Reverse the launchMotorTwo, since launchMotorTwo
        //and launchMotorOne are both on the same shaft, with opposite
        //input directions, this should make both motors spin the shaft
        //in the same direction.
        launchMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        //launchMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    //Sets up the color sensors
    //we are using in the launcher system
    /*private void initColorSensors() {

        //Intialize the launcher color sensor from
        //the hardware map.
        launcherColorSensor = opMode.hardwareMap.get(ColorSensor.class, "launcherColorSensor");
    }*/


    //Moves the shooter input servo
    //to feed a ball into the shooter
    public void inputIntoShooter() {
        shooterInput.setPower(1.0);
    }

    //Stops the servo feeding balls
    //into the shooter
    public void stopInputIntoShooter() {
        shooterInput.setPower(0.0);
    }


    //Moves the shooter input servo
    //to pivot so that the input is closed,
    //so nothing should be able to be shot out
    //of the launcher.
    //public void closeInputIntoShooter() {
    //    shooterInput.setPosition(shooterInputClosedPosition);
    //}


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
    /*public int launcherColorSensor_getColor(String color) {

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
    }*/
}
