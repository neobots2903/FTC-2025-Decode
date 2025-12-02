package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="LEDTest", group="Linear OpMode")
public class DummyLEDTester extends LinearOpMode {

    LEDSystem led;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        led = new LEDSystem(this);

        led.setIndicatorColor("PURPLE");

        while (opModeIsActive()) {

        }

    }
}
