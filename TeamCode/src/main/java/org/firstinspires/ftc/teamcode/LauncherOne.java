/*
* This class manages the robots launching system.
* Here we control dilvery of the balls to places in the game.
* */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private DcMotorEx launchMotorOne;
    private DcMotorEx launchMotorTwo;


    //Info for the launcher motors.
    //Reguarding ticks per rotation,
    //RPM to run the shooter, etc.
    int ticks_per_rotation = 28; //The total ticks for 1 rotation of the fly-wheel.
    //The RPM to run the shooter at for a far shot
    int shooter_rpm_far = 4000;
    //The shooter rpm in ticks per second for the far shot
    int shooter_rpm_in_ticks_far = (ticks_per_rotation * shooter_rpm_far) / 60;
    //The rpm for a close shot.
    int shooter_rpm_close = 3500;
    //The shooter rpm in ticks per second for the close shot
    int shooter_rpm_in_ticks_close = (ticks_per_rotation * shooter_rpm_close) / 60;





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
        launchMotorOne = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorOne");
        launchMotorTwo = opMode.hardwareMap.get(DcMotorEx.class, "launchMotorTwo");

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

    /*
    * Run the launcher at the set
    * RPM defined by arg1 "rpm"
    * */
    public void runLaucnherAtRPM(int rpm) {
        launchMotorOne.setVelocity(launcherRpmToTicks(rpm));
        launchMotorTwo.setVelocity(launcherRpmToTicks(rpm));
    }


    /*
    * Takes a argument, "rpm"
    * and converts it into ticks and returns
    * the value. This is useful for setting and running
    * motors at a sepecific rpm for the launcher.
    * */
    public int launcherRpmToTicks(int rpm) {
        return (ticks_per_rotation * rpm) / 60;
    }

    //Kills the launcher, stops all motors
    //at 0.0 power.
    //launcherMotorOne and launcherMotorTwo
    //will stop (0 power)
    public void killLauncher() {
        launchMotorOne.setPower(0.0);
        launchMotorTwo.setPower(0.0);
    }



    //Returns the current of the
    //launcher. Converts current tick velocity
    //to rpm.
    public int getRPM() {
        double rpm = 0;

        rpm = (launchMotorOne.getVelocity() * 60) / ticks_per_rotation;

        return (int)rpm;
    }


    //Runs the launcher at the set RPM for a far shot
    //for consistent launches
    public void runLauncherAtSetRPM_far() {

        //Run both the motors at the set velocity (shooter_rpm)
        launchMotorOne.setVelocity(shooter_rpm_in_ticks_far);
        launchMotorTwo.setVelocity(shooter_rpm_in_ticks_far);

    }

    //Runs the launcher at the set RPM for a close shot
    //for consistent launches
    public void runLauncherAtSetRPM_close() {

        //Run both the motors at the set velocity (shooter_rpm)
        launchMotorOne.setVelocity(shooter_rpm_in_ticks_close);
        launchMotorTwo.setVelocity(shooter_rpm_in_ticks_close);

    }


    //stops the launcher at the set RPM
    //for consistent launches; Sets velocity
    //of the shooter motors to 0.
    public void stopLauncherAtSetRPM() {

        //Run both the motors at the set velocity (shooter_rpm)
        launchMotorOne.setVelocity(0.0);
        launchMotorTwo.setVelocity(0.0);

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
