package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

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
    public DcMotorEx intakeMotor;

    //Motor for rotating the indexer
    private DcMotor indexerMotor;

    //The color sensor in the LAUNCH Position
    //of the Indexer to detect what is
    //in said position (Green, Purple, nothing, etc)
    ColorSensor LAUNCH_colorSensor;

    //The LED control system
    //for switching LED colors
    //----
    //Mainly used for indicating
    //whats at the top of the index to
    //be pushed to the shooter (Whats
    //in LAUNCH position)
    LEDSystem LED;

    //The indexes of the intake/ball storage
    //drum. We will set all these in "initIndexSystem()"
    //----
    //indexHold -> Temporary holds an index for logic and computation when cycling, etc.
    private ArrayList<Index> indexes = new ArrayList<Index>();
    private Index indexHold; //Temporary holds an index for logic and computation when cycling, etc.

    //The indexers position to be at,
    //the indexer is set to run to this position
    //to more the index forward, increment by "rotateIndexTicks"
    //or decrement for backwards.
    private int indexerTicks = 0;

    //Ticks to rotate the indexer by 1/3 rotation for the next ball.
    private int rotateIndexTicks = 96;

    //If true, the kickers engaged
    //to input a ball into the shooter
    //and we can't rotate the indexer.
    boolean kickerEngaged = false;

    //The servo for the kicker to input into
    //the launcher from the indexer.
    Servo kicker;

    //Constructor
    public indexIntakeSystem(OpMode opMode) {

        //Intialize the opmode within this class
        //so we have access to the hardware map,
        //telemetry and other IO and functions
        //within the FTC control suite.
        this.opMode = opMode;

        //Intialize the LED system
        LED = new LEDSystem(opMode);

        //Intialize all motors
        //so they can be used. Intialize
        //them with the hardware map.
        initMotors();

        //Intailize all servos so
        //we can use them.
        //Apply them to the hardware
        //map, run setups, etc.
        initServos();

        //Intialize all sensors,
        //cameras, etc
        initSensors();

        //Intialize the index system.
        //This will set all indexes as unknown and
        //set each on to a possition.
        initIndexSystem();
    }


    //Turns the robots on board
    //LEDs to the current color
    //of the ball next in the shooter.
    public void showNextBallStatus() {

        String LAUNCH_color = "";

        //For each ball in the indexes array,
        //we will check for which one is next to enter
        //the shooter if the kicker is activated.
        //(At state Index.currentPosition.Position.LAUNCH)
        for (Index ball : indexes) {

            //If the ball is at the launch position,
            //determine its color and set LED color correctly.
            if (ball.currentPosition == Index.Position.LAUNCH) {

                //Set the LED color based on the ball in the index
                LAUNCH_color = determineLAUNCHColor();

                //Set the LED color based on what
                //we detected from the color sensor
                //system for our indexer "determineLAUNCHColor()"
                //from the LAUNCH position.
                //------
                //We will also set the ball state for the
                //index to be what we detected (purple, green, etc)
                if (LAUNCH_color == "PURPLE") {
                    LED.setIndicatorColor("PURPLE");
                    ball.ballState = Index.BallStates.PURPLE;
                } else if (LAUNCH_color == "GREEN") {
                    LED.setIndicatorColor("GREEN");
                    ball.ballState = Index.BallStates.GREEN;
                }  else if (LAUNCH_color == "EMPTY") {
                    LED.setIndicatorColor("EMPTY");
                    ball.ballState = Index.BallStates.EMPTY;
                } else {
                    LED.setIndicatorColor("EMPTY");
                    ball.ballState = Index.BallStates.UNKNOWN;
                }
            }
        }
    }

    //Returns the status of the object in the indexer
    //-------
    //Returns:
    //
    //"PURPLE" -> Purple color ball or object in LAUNCH Position
    //
    //"GREEN" -> Green color ball or object in LAUNCH Position
    //
    //"EMPTY" -> Nothing is there in LAUNCH Position
    //
    //"UNKNOWN" -> We don't know whats in the LAUNCH Position
    private String determineLAUNCHColor() {

        //The color we detected.
        String colorDetected = "";

        //If we see values closer to purple
        //from the color sensor, then detect purple
        if (LAUNCH_colorSensor.red() > 50 && LAUNCH_colorSensor.blue() > 50) {
            colorDetected = "PURPLE";
        }

        //If we see values closer to green
        //from the color sensor, then detect green
        if (LAUNCH_colorSensor.green() > 70) {
            colorDetected = "GREEN";
        }

        //If we didn't see Green or Purple,
        //then just say its empty.
        if (colorDetected == "") {
            colorDetected = "EMPTY";
        }

        return colorDetected;
    }


    private void initServos() {

        //Intialize the kicker servo for
        //inputing balls from the
        //indexer into the launcher.
        kicker = opMode.hardwareMap.get(Servo.class, "kicker");

    }

    //Intializes all sensors (color sensors, cameras, etc)
    private void initSensors() {

        LAUNCH_colorSensor = opMode.hardwareMap.get(ColorSensor.class, "clrSensor");

    }

    //Intializes the indexes of the drum
    //so we can figure out where stuff is
    private void initIndexSystem() {

        //Create 3 new indexes.
        for (int i = 0; i < 3; i++) {
            indexes.add(new Index());
        }

        //Setup the first index
        indexes.get(0).ballState = Index.BallStates.UNKNOWN;
        indexes.get(0).currentPosition = Index.Position.INTAKE;

        //Setup the second index
        indexes.get(1).ballState = Index.BallStates.UNKNOWN;
        indexes.get(1).currentPosition = Index.Position.OFFHAND;

        //Setup the third index
        indexes.get(2).ballState = Index.BallStates.UNKNOWN;
        indexes.get(2).currentPosition = Index.Position.LAUNCH;

    }


    //Push a ball into the shooter
    //with the kicker servo,
    //also prevents the indexer
    //from rotating until
    //"kickerEngaged" is false.
    public void inputBall() {

        //Set the kicker to kick
        //the ball into the launcher and set
        //it as being engaged so
        //that we don't break the kicker by preventing
        //the indexer from rotating from teleop
        //inputs, because "kickerEngaged" = true
        kicker.setPosition(1.0);
        kickerEngaged = true;
    }

    //Stop inputing a ball into the launcher
    //and get ready for indexing again.
    public void stopInputtingBall() {

        //Move the kicker back to rest
        //position, so we can rotate
        //the indexer.
        kicker.setPosition(0.0);
        kickerEngaged = false;
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
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Logic to flip all the indexs positions to what they should be
        if (indexes.get(0).currentPosition == Index.Position.INTAKE) {
            indexes.get(0).currentPosition = Index.Position.OFFHAND;
            indexes.get(1).currentPosition = Index.Position.LAUNCH;
            indexes.get(2).currentPosition = Index.Position.INTAKE;
        } else if (indexes.get(0).currentPosition == Index.Position.OFFHAND) {
            indexes.get(0).currentPosition = Index.Position.LAUNCH;
            indexes.get(1).currentPosition = Index.Position.INTAKE;
            indexes.get(2).currentPosition = Index.Position.OFFHAND;
        } else if (indexes.get(0).currentPosition == Index.Position.LAUNCH) {
            indexes.get(0).currentPosition = Index.Position.INTAKE;
            indexes.get(1).currentPosition = Index.Position.OFFHAND;
            indexes.get(2).currentPosition = Index.Position.LAUNCH;
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
        indexerMotor.setTargetPosition(0);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //REVERSE MOTORS
        //Reverse any motors if needed.
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        intakeMotor.setPower(1.0);
    }


    //Kills the intake by setting its
    //power to 0.0
    public void killIntake() {
        intakeMotor.setPower(0.0);
    }

}
