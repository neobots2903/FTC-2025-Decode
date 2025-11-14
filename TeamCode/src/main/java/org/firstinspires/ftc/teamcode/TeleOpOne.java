package org.firstinspires.ftc.teamcode;

/*
* Main Class for FTC 2024 (Into The Deep); Entry point for robot.
* Class manages inputs from the controller; Controls teleop.
*
* By Colton Paul Badock - 2024 (Neobots).
*
*
**/

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@TeleOp(name="TeleOp", group="Linear OpMode")
public class TeleOpOne extends LinearOpMode {


    //Variables for gamepad1, reguarding how
    //far left or right the sticks have moved from
    //dezone (center) (0, 0)
    double gamePadOne_LeftStick_X = 0;
    double gamePadOne_LeftStick_Y = 0;
    double gamePadOne_RightStick_X = 0;

    //Total time to wait to prevent debounce in frames
    //for various systems.
    int debounceWeightTime_forShooter = 30; //Debounce wait time for the shooter
    int debounceWaitTime_forInput = 30; //Debounce wait time for the input

    //The total cycles that have passed since
    //the shooter inputted another ball.
    //In frames (app cycles).
    //Used to prevent debounce for running this system.
    int cyclesSinceInputToggled = 0;


    //Launcher variable, if this is true,
    //the launcher is actively running. If false,
    //then its not. We use this when determining
    //if we want to shut off or power on the launcher,
    //because y is used as a toggle and we use the same button
    //"y" to toggle the motor on or off.
    boolean launcherRunning = false;

    boolean inputRunning = false;

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

    //Instance of the robot
    Robot9330 robot;

    //FTC dashboard for camera streaming
    FtcDashboard dashboard;

    //Onboard camera opencv for FTC dashboard stream
    OpenCvWebcam camera;

    @Override
    public void runOpMode() {
        robot = new Robot9330(this); //Instance of our robot and its hardware.
        
        waitForStart(); //Wait until Init on Telemetry is hit before starting the robot.

        //Instance of FTC dashboard for debugging and other logs
        dashboard = FtcDashboard.getInstance();

        //System to stream camera to the FTC dashboard
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera_one"));
        dashboard.startCameraStream(camera, 2);

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

            //Controls for the launcher motors. WHen 'a' is held,
            //we will run the launcher for a far shot.
            if (gamepad2.a) {

                //Run the robot launcher.
                robot.launcher.runLauncherAtSetRPM_far();

            } else if (gamepad2.a != true && gamepad2.right_trigger == 0.0 && gamepad2.x != true && gamepad2.left_trigger == 0.0) {

                //Stop the launcher from running.
                robot.launcher.stopLauncherAtSetRPM();

            }


            //Controls for the launcher motors. WHen 'a' is held,
            //we will run the launcher for a close shot.
            if (gamepad2.x) {

                //Run the robot launcher.
                robot.launcher.runLauncherAtSetRPM_close();

            } else if (gamepad2.x != true && gamepad2.right_trigger == 0.0 && gamepad2.a != true && gamepad2.left_trigger == 0.0) {

                //Stop the launcher from running.
                robot.launcher.stopLauncherAtSetRPM();

            }


            //Run the robots launcher system using the right
            //trigger on the operator remote.
            if (gamepad2.right_trigger > 0.0) {
                robot.runLauncher(gamepad2.right_trigger);
            }


            if (gamepad2.left_trigger > 0.0) {
                robot.launcher.runLauncher(-gamepad2.left_trigger);
            }

            //Control system for the input into the shooter.
            //Allows the operator to input balls into the shooter
            if (gamepad2.b && robot.launcher.getRPM() > 500) {
                robot.runShooterInput();
            } else if (gamepad2.b != true) {
                robot.killShooterInput();
            }


            //Print the IDs of all april tags we can see
            //into the telemtry.
            printOutAllTagIdsVisible();

            //Print the power applied to the shooter manually
            telemetry.addData("Launcher trigger power: ", gamepad2.right_trigger);
            //Print the launchers current RPM
            telemetry.addData("Launchers current RPM: ", robot.launcher.getRPM());

            //Print servo data for debugging
            telemetry.addData("Servo Data: ", robot.launcher.shooterInput.getPortNumber());
            telemetry.addData("Servo Data: ", robot.launcher.shooterInput.getManufacturer());

            speedToggle(); //Allow the driver to change the speed multiplier with the bumpers.


            //Update the total cycles that have
            //passed since we ran the input
            //for the shooter to input a ball
            cyclesSinceInputToggled++;

            //Update the cycles that have passed
            //since we toggled the launcher. Just
            //add in one application cycle, since
            //another application loop has completed.
            cyclesSinceLauncherToggled++;

            //Update the telemetry output each cycle.
            telemetry.update();
        }
    }

    //Prints the ids of all tags we can see
    public void printOutAllTagIdsVisible() {
        //Print the IDs of all april tags we can see
        //into the telemtry.
        List<Integer> ids = robot.camera. getAllAprilTag_ids();

        //String to store all detected tag ids
        //inside of.
        String tagsSeen = "";

        //Take the list of all april tag ids we have,
        //print each of them into a string then print it out
        for (int tagIds : ids) {
            tagsSeen += tagIds + ", ";
        }

        //Print list of all visible tags into
        //telemetry stream.
        telemetry.addData("Tag IDs seen: ", tagsSeen);
    }

    //Allows the operator to tune the speed on the fly.
    public void speedToggle() {

        //Based on inputs from driver on the bumpers, toggle the speed of the robot.
        if (gamepad1.left_bumper == true && timeSinceLastSpeedChange.milliseconds() > 200) {
            if (speedMultiplier > 0.4) {
                speedMultiplier -= 0.2; //Change the speed multiplier.
                timeSinceLastSpeedChange = new ElapsedTime(); //Reset the timer since the last speed change.
            }
        } else if (gamepad1.right_bumper == true && timeSinceLastSpeedChange.milliseconds() > 200) {
            if (speedMultiplier < 1.0) {
                speedMultiplier += 0.2; //Change the speed multiplier.
                timeSinceLastSpeedChange = new ElapsedTime(); //Reset the timer since the last speed change.
            }
        }

        telemetry.addData("Speed multiplier: ", speedMultiplier);
        telemetry.update();
    }
}
