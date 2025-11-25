/*
 *This is the auto for when we start facing away from the
 * obelisk on the blue alliance.
 * */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class blueAuto_awayFromObelisk extends LinearOpMode {

    awayObeliskAuto auto;

    @Override
    public void runOpMode() throws InterruptedException {

        auto = new awayObeliskAuto(this, "BLUE");

        waitForStart();

        auto.runAuto();

    }
}
