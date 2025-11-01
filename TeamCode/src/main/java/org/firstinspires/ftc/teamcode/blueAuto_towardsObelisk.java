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
public class blueAuto_towardsObelisk extends LinearOpMode {


    //Poses;
    //These are positions and orientations/locations for the robot to reach
    Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));

    //Vectors
    Vector2d shootingPosition = new Vector2d(-20, -15); //The position to shot from.
    double firingPositionRotation = 2.0; //The heading to aim for the goal to score from the firing position

    //Instance of the camera manager.
    //Allows for april tag vision.
    CameraManager camera;


    //Trajectory actions, lists of movements an actions
    //we will follow in auto.
    //Faces the robot toward the goal
    //from the starting zone so we can line
    TrajectoryActionBuilder toFiringPos;


    //Actions, groups of final built trajectories
    Action toFiringPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        //Create an instance of the drive base
        //for the roadrunner system
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //Setup the "toFiringPos" trjectory to get the bot to the firing positon.
        //
        //Trajectories:
        //
        //.strafeTo(shootingPosition) -> Strafe the robot to the roboting position
        //with no rotations.
        toFiringPos = drive.actionBuilder(beginPose).strafeTo(shootingPosition).turnTo(Math.toRadians(firingPositionRotation));
        toFiringPosition = toFiringPos.build(); //Build the action, so we can run it.

        //Intialize the camera manager for
        //april tag vision
        camera = new CameraManager(this);

        //Wait for the start to be hit on telemetry
        waitForStart();


        //Start the autonomous
        if (opModeIsActive()) {

            telemetry.addData("Auto", "Running Auto");
            telemetry.update();

            //Run the action to get the bot to the
            //firing position.
            Actions.runBlocking(new SequentialAction(toFiringPosition));

            drive.rightBack.setPower(1.0);


            telemetry.addData("Auto", "Auto Finished");
            telemetry.update();
        }
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
