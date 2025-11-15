package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class redAuto_towardsObelisk extends LinearOpMode {

    towardsObeliskAuto auto;

    @Override
    public void runOpMode() throws InterruptedException {

        //Intialize the robots hardware in the constructor
        auto = new towardsObeliskAuto(this, "RED");

        //Wait till auto is started
        waitForStart();

        //Run the auto for the blue side
        auto.runAuto();

    }
}