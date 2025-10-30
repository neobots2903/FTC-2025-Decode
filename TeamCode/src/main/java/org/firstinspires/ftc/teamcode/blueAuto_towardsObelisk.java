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
        if (opModeIsActive()) {

            telemetry.addData("April tag position", camera.findTag(24).ftcPose);
            telemetry.update();

        }
    }



}
