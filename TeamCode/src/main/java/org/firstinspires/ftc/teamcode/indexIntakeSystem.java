package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
* The "indexIntakeSystem".
* This class manages intaking and indexing for the shooter.
* Everything here is used to feed balls into the robot for storage and so
* we can pick which ball we will fire.
* */
public class indexIntakeSystem {

    //Instance of opmode for
    //hardware control and mamangement
    //of telemetry.
    private OpMode opMode;

    //Intake motors for intaking ground
    //balls into the indexer.
    private DcMotorEx intakeMotor;

    //Motor for rotating the indexer
    private DcMotor indexerMotor;

    //The indexes of the intake/ball storage
    //drum. We will set all these in "initIndexSystem()"
    //----
    //indexHold -> Temporary holds an index for logic and computation when cycling, etc.
    private Index[] indexes = new Index[2];
    private Index indexHold; //Temporary holds an index for logic and computation when cycling, etc.

    //The indexers position to be at,
    //the indexer is set to run to this position
    //to more the index forward, increment by "rotateIndexTicks"
    //or decrement for backwards.
    private int indexerTicks = 0;

    private int rotateIndexTicks = 96;

    //Constructor
    public void indexIntakeSystem(OpMode opMode) {

        //Intialize the opmode within this class
        //so we have access to the hardware map,
        //telemetry and other IO and functions
        //within the FTC control suite.
        this.opMode = opMode;

        //Intialize all motors
        //so they can be used. Intialize
        //them with the hardware map.
        initMotors();

    }

    //Intializes the indexes of the drum
    //so we can figure out where stuff is
    private void initIndexSystem() {

        //Setup the first index
        indexes[0].ballState = Index.BallStates.UNKNOWN;
        indexes[0].currentPosition = Index.Position.INTAKE;

        //Setup the second index
        indexes[1].ballState = Index.BallStates.UNKNOWN;
        indexes[1].currentPosition = Index.Position.OFFHAND;

        //Setup the third index
        indexes[2].ballState = Index.BallStates.UNKNOWN;
        indexes[2].currentPosition = Index.Position.LAUNCH;

    }


    //Cycles the intake forward one time
    public void cycleIntake() {

        //ID: 892349827498274
        //Increment the indexers current position by
        //one-third full rotation (rotateIndexTicks)
        //to get the next index up.
        indexerTicks += rotateIndexTicks;

        //Runs the indexer position to
        //the tick position it should be at
        //after we calculated an addition rotation
        //at ID: 892349827498274
        indexerMotor.setTargetPosition(indexerTicks);

        //Logic to flip all the indexs positions to what they should be
        if (indexes[0].currentPosition == Index.Position.INTAKE) {
            indexes[0].currentPosition = Index.Position.OFFHAND;
            indexes[1].currentPosition = Index.Position.LAUNCH;
            indexes[2].currentPosition = Index.Position.INTAKE;
        } else if (indexes[0].currentPosition == Index.Position.OFFHAND) {
            indexes[0].currentPosition = Index.Position.LAUNCH;
            indexes[1].currentPosition = Index.Position.INTAKE;
            indexes[2].currentPosition = Index.Position.OFFHAND;
        } else if (indexes[0].currentPosition == Index.Position.LAUNCH) {
            indexes[0].currentPosition = Index.Position.INTAKE;
            indexes[1].currentPosition = Index.Position.OFFHAND;
            indexes[2].currentPosition = Index.Position.LAUNCH;
        }

    }


    //When called, all motors onboard the index/intake system
    //will be reset and reintialized.
    private void initMotors() {

        //Intialize the intake motor with
        //opModes hardware map so we can
        //see it in the configure
        //and configure it.
        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        //Intialize the motor for rotating
        //the indexer.
        //-----
        //Set the motor to run to position
        //mode whilst reseting the encoder to 0 ticks
        indexerMotor = opMode.hardwareMap.get(DcMotor.class, "indexMotor");
        indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //REVERSE MOTORS
        //Reverse any motors if needed.
        //intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Runs the intake motor for intake
    //The motor will intake until "killIntake()"
    //is called.
    public void runIntake() {

        //Runs the intake at power, velocity
        //should not matter very much since we
        //are simply sucking the ball in and
        //don't need a trajectory.
        intakeMotor.setPower(0.3);
    }


    //Kills the intake by setting its
    //power to 0.0
    public void killIntake() {
        intakeMotor.setPower(0.0);
    }

}
