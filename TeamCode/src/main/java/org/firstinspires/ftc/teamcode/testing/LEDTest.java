package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class LEDTest extends OpMode {

    RevBlinkinLedDriver lights;

    public void init() {
//LED sequence
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
    }

    public void loop() {


        if (gamepad1.a) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        } else if (gamepad1.b) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (gamepad1.x) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (gamepad1.y) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }
    }
}