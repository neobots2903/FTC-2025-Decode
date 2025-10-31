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

    //Instance of the camera manager.
    //Allows for april tag vision.
    CameraManager camera;


    @Override
    public void runOpMode() throws InterruptedException {

        //Create an instance of the drive base
        //for the roadrunner system
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //Intialize the camera manager for
        //april tag vision
        camera = new CameraManager(this);

        //Wait for the start to be hit on telemetry
        waitForStart();


        //Start the autonomous
        while (opModeIsActive()) {

            AprilTagDetection tag = camera.findTag(24);

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
            telemetry.update();

        }
    }



}
