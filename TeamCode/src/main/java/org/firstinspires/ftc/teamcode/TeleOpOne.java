package org.firstinspires.ftc.teamcode;

/*
* Main Class for FTC 2024 (Into The Deep); Entry point for robot.
* Class manages inputs from the controller; Controls teleop.
*
* By Colton Paul Badock - 2024 (Neobots).
*
*
**/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="Linear OpMode")
public class TeleOpOne extends LinearOpMode {


    //Variables for gamepad1, reguarding how
    //far left or right the sticks have moved from
    //dezone (center) (0, 0)
    double gamePadOne_LeftStick_X = 0;
    double gamePadOne_LeftStick_Y = 0;
    double gamePadOne_RightStick_X = 0;


    //Launcher variable, if this is true,
    //the launcher is actively running. If false,
    //then its not. We use this when determining
    //if we want to shut off or power on the launcher,
    //because y is used as a toggle and we use the same button
    //"y" to toggle the motor on or off.
    boolean launcherRunning = false;


    //Launcher Variable, this represents
    //the application cycles (frames) of the
    //teleop loop that have passed since the
    //launcher was toggled on or off.
    //This prevents the d-bounce from
    //the y-button all in two frames (I.E.
    //the user has time to release the Y button,
    //so that it doesn't switch the launcher straight
    //off).
    int cyclesSinceLauncherToggled = 0;


    //SPEED MULTIPLIER
    double speedMultiplier = 1.0; //Multiplier of the speed for the robots driving.
    ElapsedTime timeSinceLastSpeedChange = new ElapsedTime(); //Time since the last speed input was entered by the driver, used mainly for debounce.


    @Override
    public void runOpMode() {
        Robot9330 robot = new Robot9330(this); //Instance of our robot and its hardware.
        
        waitForStart(); //Wait until Init on Telemetry is hit before starting the robot.
        
        //Infinite application loop.
        while (opModeIsActive()) {
            
            //Get inputs from the gamepad/controller and assign them to the variables
            gamePadOne_LeftStick_X = gamepad1.left_stick_x;
            gamePadOne_LeftStick_Y = -gamepad1.left_stick_y;
            gamePadOne_RightStick_X = gamepad1.right_stick_x;
            
            robot.move(gamePadOne_LeftStick_X * speedMultiplier, gamePadOne_LeftStick_Y * speedMultiplier, gamePadOne_RightStick_X * speedMultiplier); //Control robot movement based on controller inputs.

            //Check to see if the driver wants to reset the robots headless (YAW). Same as starting intializing the robot for the drive system.
            if (gamepad1.x) {
                robot.resetHeadless();
            }

            //Controls for the launcher motors. WHen 'y' is pressed,
            //we will run the launcher, pressing it again will kill
            //the launcher motors. (Acts like a toggle)
            if (gamepad2.y && launcherRunning == false && cyclesSinceLauncherToggled > 30) {

                //Run the robot launcher.
                robot.runLauncher();

                //Set the launcherRunning to true,
                //so that we know the launcher is running.
                launcherRunning = true;

                //Reset the cycles since the launcher toggle
                //button was pressed, to prevent d-bounce.
                cyclesSinceLauncherToggled = 0;

            } else if (gamepad2.y && launcherRunning == true && cyclesSinceLauncherToggled > 10) {

                //Stop the launcher from running.
                robot.killLauncher();

                //Set the launcherRunning to false,
                //so that we know the launcher is not running.
                launcherRunning = false;

                //Reset the cycles since the launcher toggle
                //button was pressed, to prevent d-bounce.
                cyclesSinceLauncherToggled = 0;
            }

            speedToggle(); //Allow the driver to change the speed multiplier with the bumpers.

            //Update the cycles that have passed
            //since we toggled the launcher. Just
            //add in one application cycle, since
            //another application loop has completed.
            cyclesSinceLauncherToggled++;
        }
    }


    //Allows the operator to tune the speed on the fly.
    public void speedToggle() {

        //Based on inputs from driver on the bumpers, toggle the speed of the robot.
        if (gamepad1.left_bumper == true && timeSinceLastSpeedChange.milliseconds() > 200) {
            if (speedMultiplier > 0.2) {
                speedMultiplier -= 0.2; //Change the speed multiplier.
                timeSinceLastSpeedChange = new ElapsedTime(); //Reset the timer since the last speed change.
            }
        } else if (gamepad1.right_bumper == true && timeSinceLastSpeedChange.milliseconds() > 200) {
            if (speedMultiplier < 1.0) {
                speedMultiplier += 0.2; //Change the speed multiplier.
                timeSinceLastSpeedChange = new ElapsedTime(); //Reset the timer since the last speed change.
            }
        }
    }
}
