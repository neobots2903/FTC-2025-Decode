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
    
    double gamePadOne_LeftStick_X = 0;
    double gamePadOne_LeftStick_Y = 0;
    double gamePadOne_RightStick_X = 0;
    boolean intakeIsRunning = false; //If false, the intake is stopped.


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

            speedToggle(); //Allow the driver to change the speed multiplier with the bumpers.

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
