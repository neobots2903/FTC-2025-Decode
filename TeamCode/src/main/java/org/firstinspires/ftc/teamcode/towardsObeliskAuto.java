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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class towardsObeliskAuto {

    //Auto constants
    AutoConstants constants = new AutoConstants();

    //Vectors
    Vector2d shootingPosition;
    //Originally: 52.00
    double firingPositionRotation;
    Vector2d parkPosition;

    //OpMode from auto classes
    //for hardware interaction
    OpMode opMode;

    //The side/team we are on for the auto
    //BLUE = Blue side/alliance
    //RED = Red side/alliance.
    String side = "";


    //Poses;
    //These are positions and orientations/locations for the robot to reach
    //Starting position for the robot (0, 0, rotation = 0)
    Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
    //The position to shoot the balls from.
    //Equivalent of the rotation "firingPositionRotation" at the "shootingPosition"
    Pose2d shootingPositionPose;


    //The RPM to shoot balls at
    int shooterRPM;
    //The threshhold to detect a ball was launched
    //I.E. if 100, then once we are 100 RPM down from the ideal
    //launch RPM, a ball was probably launched
    int rpmThreshold;


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

    //Constructor;
    //Intialize onboard systems for the robot
    public towardsObeliskAuto(OpMode opMode, String side) {

        //Set the opmode and the side/alliance
        //passed from the auto class.
        this.opMode = opMode;
        this.side = side;

        //Create an instance of the drive base
        //for the roadrunner system
        MecanumDrive drive = new MecanumDrive(opMode.hardwareMap, beginPose);

        //Sets the auto constants/variables
        //based on if we are RED or BLUE
        //alliance
        initAutoConstants();

        //Setup the "toFiringPos" trjectory to get the bot to the firing positon.
        //
        //Trajectories:
        //
        //.strafeTo(shootingPosition) -> Strafe the robot to the roboting position
        //with no rotations.
        toFiringPos = drive.actionBuilder(beginPose).strafeTo(shootingPosition).turnTo(Math.toRadians(firingPositionRotation));
        toFiringPosition = toFiringPos.build(); //Build the action, so we can run it.

        //Build the action to park the robot
        //to the human player zone.
        toParkingPos = drive.actionBuilder(shootingPositionPose).strafeTo(parkPosition);
        toParkingPosition = toParkingPos.build();

        //Intialize the camera manager for
        //april tag vision
        camera = new CameraManager(opMode);

        //Setup/intialize the launcher
        launcher = new LauncherOne(opMode);
    }


    //Sets the auto constants/variables
    //based on if we are RED or BLUE
    //alliance
    private void initAutoConstants() {

        //If the side is blue, set variables, vectors for
        //the blue side
        if (side == "BLUE") {

            shootingPosition = new Vector2d(constants.blue_towardsOblesk_shootingPosition_x, constants.blue_towardsOblesk_shootingPosition_y); //The position to shot from.
            firingPositionRotation = constants.blue_towardsOblesk_firingPositionRotation; //The heading to aim for the goal to score from the firing position
            parkPosition = new Vector2d(constants.blue_towardsOblesk_parkPosition_x, constants.blue_towardsOblesk_parkPosition_y); //The position to park the robot at (in the human player zone)
            shooterRPM = constants.shooterRPM; //The RPM to shoot balls at
            rpmThreshold = constants.ballDetectedThreshhold; //The threshhold to detect a ball was launched. I.E. if 100, then once we are 100 RPM down from the ideal; launch RPM, a ball was probably launched
            shootingPositionPose = new Pose2d(shootingPosition.x, shootingPosition.y, Math.toRadians(firingPositionRotation));

        } else if (side == "RED") {
            shootingPosition = new Vector2d(constants.red_towardsOblesk_shootingPosition_x, constants.red_towardsOblesk_shootingPosition_y); //The position to shot from.
            firingPositionRotation = constants.red_towardsOblesk_firingPositionRotation; //The heading to aim for the goal to score from the firing position
            parkPosition = new Vector2d(constants.red_towardsOblesk_parkPosition_x, constants.red_towardsOblesk_parkPosition_y); //The position to park the robot at (in the human player zone)
            shooterRPM = constants.shooterRPM; //The RPM to shoot balls at
            rpmThreshold = constants.ballDetectedThreshhold; //The threshhold to detect a ball was launched. I.E. if 100, then once we are 100 RPM down from the ideal; launch RPM, a ball was probably launched
            shootingPositionPose = new Pose2d(shootingPosition.x, shootingPosition.y, Math.toRadians(firingPositionRotation));

        }
    }

    //Runs the auto.
    //Same as "if (opMode.isActive())"
    public void runAuto() throws InterruptedException {

        //Run the action to get the bot to the
        //firing position.
        Actions.runBlocking(new SequentialAction(toFiringPosition));

        //Run the launcher at the set RPM for the close shot
        //Fire all 3 balls, then return here to park.
        fireThreeBalls();

        //Leave the shooting zone to dodge penalty
        Actions.runBlocking(new SequentialAction(toParkingPosition));
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

            opMode.telemetry.addData("RPM: ", launcher.getRPM());
            opMode.telemetry.update();

            //If the launcher motors are spinning within 100 RPM of the
            //shooters resquested power for the shot, begin
            //injecting a ball, as by the time its in, it will be at RPM.
            if (launcher.getRPM() > shooterRPM - rpmThreshold) {

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
                while (launcher.getRPM() > shooterRPM - rpmThreshold) {
                    launcher.inputIntoShooter();
                    opMode.telemetry.addData("RPM: ", launcher.getRPM());
                    opMode.telemetry.update();
                }

                //Kill the launcher, we feel below "shooterRPM" by 100
                //RPM, likely having shot the ball.
                launcher.stopInputIntoShooter();

                //Register we shoot a ball.
                ballsShot++;

                opMode.telemetry.addData("Balls shoot: ", ballsShot);
                opMode.telemetry.addData("RPM: ", launcher.getRPM());
                opMode.telemetry.update();
            }

            //If we shot the last ball,
            //lets wait 500 milliseconds (1/2 second)
            //so that we don't kill the shooter earlier as the
            //ball is feeding through
            if (ballsShot == 3) {
                try {
                    Thread.sleep(500);
                } catch (Exception e) {}
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
            opMode.telemetry.addData("X", tag.ftcPose.x);
            opMode.telemetry.addData("Y", tag.ftcPose.y);
            opMode.telemetry.addData("Z", tag.ftcPose.z);
            opMode.telemetry.addData("YAW", tag.ftcPose.yaw);
            opMode.telemetry.addData("ROLL", tag.ftcPose.roll);
            opMode.telemetry.addData("BEARING", tag.ftcPose.bearing);
            opMode.telemetry.addData("PITCH", tag.ftcPose.pitch);
            opMode.telemetry.addData("RANGE", tag.ftcPose.range);
            opMode.telemetry.addData("ELEVATION", tag.ftcPose.elevation);
            opMode.telemetry.addData("ITS PASTA DAY", 1);
        }

        //Update the telemetry.
        opMode.telemetry.update();

    }

}
