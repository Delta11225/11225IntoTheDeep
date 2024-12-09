package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp
public class LEDTest extends OpMode {

    RevBlinkinLedDriver lights;
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    String sampleColor = "none";

    public void init() {
//LED sequence
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorv3"); //claw distance?
        sensorColor = hardwareMap.get(ColorSensor.class, "colorv3");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
    }

    public void loop() {

        if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red()) && (sensorDistance.getDistance(DistanceUnit.CM) <= 2)) {
            sampleColor = "blue";
        }
        else if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.red() > sensorColor.green()) && sensorDistance.getDistance(DistanceUnit.CM) <= 2) {
            sampleColor = "red";
        }
        else if (sensorDistance.getDistance(DistanceUnit.CM) <= 2) {
            sampleColor = "yellow";
        }
        else {
            sampleColor = "none";
            }



        if (sampleColor == "blue") {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
        } else if (sampleColor == "red") {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else if (sampleColor == "yellow") {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        }
    }
}