package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class awayObeliskAuto {

    //Poses;
    //These are positions and orientations/locations for the robot to reach
    //Starting position for the robot (0, 0, rotation = 0)
    Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));

    AutoConstants constants = new AutoConstants();

    // opMode allows access to hardware map and telemetry
    OpMode opMode;

    LauncherOne launcher;

    //Vectors of our positions to move to in the auto
    Vector2d firingPosition; //Position to fire
    Vector2d parkingPosition; //Position to park
    double firingRotation;

    // This action gets us to the firing position
    TrajectoryActionBuilder toFiringPosition;

    // holds the side we're on
    // red or blue aliance
    String side;

    Action toFiringPos;

    // Constructor
    public awayObeliskAuto(OpMode opMode, String side) {
        this.opMode = opMode;
        this.side = side;

        //Create an instance of the drive base
        //for the roadrunner system
        MecanumDrive drive = new MecanumDrive(opMode.hardwareMap, beginPose);

        launcher = new LauncherOne(opMode);

        initAutoConstants();


        // This action will get us to our firing position
        toFiringPosition = drive.actionBuilder(beginPose).strafeTo(firingPosition).turnTo(Math.toRadians(firingRotation));
        toFiringPos = toFiringPosition.build();


    }


    //set auto constants based on the side
    public void initAutoConstants() {

        if (side == "BLUE") {
            firingPosition = new Vector2d(constants.blue_awayObelisk_firingPosition_x, constants.blue_awayObelisk_firingPosition_y);
            firingRotation = constants.blue_awayObelisk_firingPosition_rotation;
        } else if (side == "RED") {
            firingPosition = new Vector2d(constants.red_awayObelisk_firingPosition_x, constants.red_awayObelisk_firingPosition_y);
            firingRotation = constants.red_awayObelisk_firingPosition_rotation;
        }

    }

    // runs the auto
    public void runAuto() throws InterruptedException {

        //Run the action to get the bot to the
        //firing position.
        Actions.runBlocking(new SequentialAction(toFiringPos));

        launcher.fireThreeBalls(constants.shooterRPM, constants.shooterRPMThreshhold);

    }

}
