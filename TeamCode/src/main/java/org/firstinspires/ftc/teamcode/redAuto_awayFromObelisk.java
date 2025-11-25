/*
 *This is the auto for when we start facing away from the
 * obelisk on the red alliance.
 * */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class redAuto_awayFromObelisk extends LinearOpMode {

    awayObeliskAuto auto;

    @Override
    public void runOpMode() throws InterruptedException {

        auto = new awayObeliskAuto(this, "RED");

        waitForStart();

        auto.runAuto();

    }
}
