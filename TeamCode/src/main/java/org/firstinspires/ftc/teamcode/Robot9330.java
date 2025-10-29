package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
* Robot hardware control; Class controls all arms, servos, motors, etc.
*
*
**/
public class Robot9330 {

    //OpMode
    OpMode opMode; //OpMode from TeleOp.


    //Motors
    DcMotor motorDriveFrontLeft; //Front Left motor for wheel base.
    DcMotor motorDriveFrontRight; //Front Right motor for wheel base.
    DcMotor motorDriveBackLeft; //Rear Left motor for wheel base.
    DcMotor motorDriveBackRight; //Rear Right motor for wheel base.



    //IMU systems.
    BNO055IMU imu; //Old IMU (Should not be used.)
    IMU newIMU; //New IMU
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation myRobotOrientation; //Holds the robots orientation from the IMU.


    //Drive System Variables
    double botHeading; //Rotation of the robot from IMU.
    double rotY; //rotation veriable for Y axis
    double rotX; //rotation veriable for X axis
    double motorPowerDenominator; //This ratio will make sure the motors keep the same power-balance, I.E. the motors will run with a similar ratio and not spin out while mechanium driving.
    double frontLeftPower, frontRightPower, backRightPower, backLeftPower; //Power for each of the motors.


    //The robots launcher system.
    //We use this system to launch balls and other objects
    //using our robot.
    LauncherOne launcher;


    //Constructor; Robot inits from here.
    public Robot9330(OpMode opMode) {
        this.opMode = opMode;
        
        //Old IMU intialization.
        //imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        //imu.initialize(parameters);
        
        //Get IMU from hardware.
        newIMU = opMode.hardwareMap.get(IMU.class, "imu");

        //Setup robot orientation information for IMU initalization; Default setup, will change if robot is comp bot.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


        //Orientation of the IMU, for initalizing the new IMU interface.
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        //Intialize new IMU interface.
        //newIMU = opMode.hardwareMap.get(IMU.class, "imu");
        //newIMU.initialize(parameters);
        newIMU.initialize(new IMU.Parameters(orientationOnRobot));
        newIMU.resetYaw(); //Resets the robots IMU to zero, so that the YAW is at 0, so when we initalize from behind the robot, out YAW is zero and the rotational measurement is based on the robots orientation at initialization

        //Init motors; Get the motor class configuraed to the hardware.
        motorDriveFrontLeft = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        motorDriveFrontRight = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        motorDriveBackLeft = opMode.hardwareMap.get(DcMotor.class, "leftBack");
        motorDriveBackRight = opMode.hardwareMap.get(DcMotor.class, "rightBack");
        //memeServo = opMode.hardwareMap.get(Servo.class, "memeServo");



        //Reverse Motors
        motorDriveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //Initalize the robots IMU orientation.
        myRobotOrientation = newIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        //Setup the robots launcher system
        launcher = new LauncherOne(opMode);

    }


    //Runs the launcher motor at the power
    //of argument 1.
    //
    //Arguments:
    //
    //power -> float of the power to run the motors at
    public void runLauncher(float power) {
        launcher.runLauncher(power);
    }

    //Kills the launcher motors by setting
    //the power to 0.0 (complete stop).
    public void killLauncher() {
        launcher.killLauncher();
    }

    //Runs the motors continuesly, as a test.
    public void runMotorsIndefentlyTest() {
        motorDriveFrontLeft.setPower(1);
        motorDriveFrontRight.setPower(1);
        motorDriveBackLeft.setPower(1);
        motorDriveBackRight.setPower(1);
    }


    //Use the shooter interface to input
    //the ball into the shooter
    public void runShooterInput() {
        launcher.inputIntoShooter();
    }

    //Use the shooter interface to kill the input
    //the ball into the shooter
    public void killShooterInput() {
        launcher.stopInputIntoShooter();
    }


    
    //Spins one motor; Used for testing.
    public void spinOneMotor() {
        motorDriveBackLeft.setPower(1);
    }
    
    
    //Robot move method. Moves the robot based on the controller stick inputs.
    public void move(double x, double y, double rx) {

        //Get the rotation of the robot as a degree from its starting position.
        //The rotation we are getting is measured in DEGREES from 360-0.
        botHeading = newIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        
        rotX = rotX * 1.1;  // Counteract imperfect strafing
        
        //Make sure all the motors keep the same power-balance/ratio? Stolen from: gmzero.
        motorPowerDenominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        
        //Drive the motors based on there rotation, position.
        frontLeftPower = (rotY + rotX + rx) / motorPowerDenominator;
        backLeftPower = (rotY - rotX + rx) / motorPowerDenominator;
        frontRightPower = (rotY - rotX - rx) / motorPowerDenominator;
        backRightPower = (rotY + rotX - rx) / motorPowerDenominator;
        
        //Drive the motors based on our
        motorDriveFrontLeft.setPower(frontLeftPower);
        motorDriveBackLeft.setPower(backLeftPower);
        motorDriveFrontRight.setPower(frontRightPower);
        motorDriveBackRight.setPower(backRightPower);
        
        //OLD MAGICAL MATH SYSTEM.
        //double botHeading = -imu.getAngularOrientation().firstAngle;
        // Magical math
        /*double botHeading = newIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); //get angular orientation for new IMU interface.
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = -(rotY - rotX - rx) / denominator; //MOTOR is flipped.
        double backLeftPower = -(rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        */ //END OLD MAGICAL MATH SYSTEM
    }


    //Resets the IMU's headless; Resets YAW to 0.
    public void resetHeadless() {
        newIMU.resetYaw();
    }

}
