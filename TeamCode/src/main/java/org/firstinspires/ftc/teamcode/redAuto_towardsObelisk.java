/*
 *This is the auto for when we start facing towards the
 * obelisk on the blue alliance.
 * */
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class redAuto_towardsObelisk extends LinearOpMode {

    //Auto constants
    AutoConstants constants = new AutoConstants();

    //Vectors
    Vector2d shootingPosition = new Vector2d(constants.red_towardsOblesk_shootingPosition_x, constants.red_towardsOblesk_shootingPosition_y); //The position to shot from.
    //Originally: 52.00
    double firingPositionRotation = constants.red_towardsOblesk_firingPositionRotation; //The heading to aim for the goal to score from the firing position
    Vector2d parkPosition = new Vector2d(constants.red_towardsOblesk_parkPosition_x, constants.red_towardsOblesk_parkPosition_y); //The position to park the robot at (in the human player zone)


    //Poses;
    //These are positions and orientations/locations for the robot to reach
    //Starting position for the robot (0, 0, rotation = 0)
    Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
    //The position to shoot the balls from.
    //Equivalent of the rotation "firingPositionRotation" at the "shootingPosition"
    Pose2d shootingPositionPose = new Pose2d(shootingPosition.x, shootingPosition.y, Math.toRadians(firingPositionRotation));


    //The RPM to shoot balls at
    int shooterRPM = constants.shooterRPM;

    //Instance of the camera manager.
    //Allows for april tag vision.
    CameraManager camera;


    //Trajectory actions, lists of movements an actions
    //we will follow in auto.
    //Faces the robot toward the goal
    //from the starting zone so we can line
    TrajectoryActionBuilder toFiringPos;

    //The action to get to the
    //"parking zone", I.E. the human
    //player area.
    TrajectoryActionBuilder toParkingPos;


    //Actions, groups of final built trajectories
    Action toFiringPosition;
    Action toParkingPosition;

    //The launcher, to run the launcher in the auto
    LauncherOne launcher;

    @Override
    public void runOpMode() throws InterruptedException {

        //Create an instance of the drive base
        //for the roadrunner system
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);



        //Flip all our "Trajectory Action Builder" sequences so
        //we can run the blues path but on the red side.
        //flipToRed();

        //Setup the "toFiringPos" trjectory to get the bot to the firing positon.
        //
        //Trajectories:
        //
        //.strafeTo(shootingPosition) -> Strafe the robot to the roboting position
        //with no rotations.
        toFiringPos = drive.actionBuilder(beginPose).strafeTo(shootingPosition).turnTo(Math.toRadians(firingPositionRotation));
        //Build the action to park the robot
        //to the human player zone.
        toParkingPos = drive.actionBuilder(shootingPositionPose).strafeTo(parkPosition);

        //Build the road runner paths
        toFiringPosition = toFiringPos.build(); //Build the action, so we can run it.
        toParkingPosition = toParkingPos.build();

        //Intialize the camera manager for
        //april tag vision
        camera = new CameraManager(this);

        //Setup/intialize the launcher
        launcher = new LauncherOne(this);

        //Wait for the start to be hit on telemetry
        waitForStart();


        //Start the autonomous
        if (opModeIsActive()) {

            //Run the action to get the bot to the
            //firing position.
            Actions.runBlocking(new SequentialAction(toFiringPosition));

            //Run the launcher at the set RPM for the close shot
            //Fire all 3 balls, then return here to park.
            //fireThreeBalls();

            launcher.runLaucnherAtRPM(shooterRPM);
            launcher.inputIntoShooter();

            try {
                Thread.sleep(10000);
            } catch (Exception e) {}


            //Leave the shooting zone to dodge penalty
            Actions.runBlocking(new SequentialAction(toParkingPosition));

            launcher.killLauncher();
        }
    }



    //This auto is written/based on the blue zone, so this
    //method will flip everything so we can run it from the
    //red side.
    private void flipToRed() {

        //Reverse our trajectories
        toFiringPos.setReversed(true);
        toParkingPos.setReversed(true);
    }


    //Fires 3 balls at close range into the score
    //zone. Waits until the launcher is up to speed
    //before shooting.
    private void fireThreeBalls() {

        //The total balls that have been shot by
        //the robot. (Not nessicairly scored)
        int ballsShot = 0;

        //Run the launcher at the set RPM
        launcher.runLaucnherAtRPM(shooterRPM);

        //Stay in this loop till
        //we have fired 3 balls.
        while (ballsShot < 3) {

            //If the launcher motors are spinning within 100 RPM of the
            //shooters resquested power for the shot, begin
            //injecting a ball, as by the time its in, it will be at RPM.
            if (launcher.getRPM() > shooterRPM - 100) {

                //We detected we are wihtin RPM to begin
                //shooting, start feeding a ball into the
                //shooter.
                launcher.inputIntoShooter();

                //While the launcher is within 100 RPM of
                //our requested launch rpm "shooterRPM",
                //keep feeding a ball in;
                //Once the RPM has dropped 100 or more RPM below
                //the requested "shooterRPM" for a shot,
                //we will exit this loop and kill the shooter
                while (launcher.getRPM() > shooterRPM - 100) {
                    launcher.inputIntoShooter();
                }

                //Kill the launcher, we feel below "shooterRPM" by 100
                //RPM, likely having shot the ball.
                launcher.stopInputIntoShooter();

                //Register we shoot a ball.
                ballsShot++;
            }
        }

        launcher.runLaucnherAtRPM(0);
        launcher.killLauncher();
    }

    //Print the pose data of the april
    //tag by the id of argument 1 (tagId).
    //
    //Arguments:
    //
    //tagId -> The ID of the tag to print the pose for (assuming its visible)
    private void printAprilTagPoseInfo(int tagId) {

        //Find the april tag by the id "tagId"
        //and store the tags instance in "tag"
        AprilTagDetection tag = camera.findTag(tagId);

        //Print all the pose positions
        //of "tag"
        if (tag != null) {
            telemetry.addData("X", tag.ftcPose.x);
            telemetry.addData("Y", tag.ftcPose.y);
            telemetry.addData("Z", tag.ftcPose.z);
            telemetry.addData("YAW", tag.ftcPose.yaw);
            telemetry.addData("ROLL", tag.ftcPose.roll);
            telemetry.addData("BEARING", tag.ftcPose.bearing);
            telemetry.addData("PITCH", tag.ftcPose.pitch);
            telemetry.addData("RANGE", tag.ftcPose.range);
            telemetry.addData("ELEVATION", tag.ftcPose.elevation);
            telemetry.addData("ITS PASTA DAY", 1);
        }

        //Update the telemetry.
        telemetry.update();

    }

}
