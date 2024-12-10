package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp
public class ClawAutoGrabTestingLT extends OpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    public Servo servo0 = null;

    double rightClawClosed = 0.1;
    double rightClawOpened = 0.4;


    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        servo0 = hardwareMap.get(Servo.class, "servo_0");
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3");
        servo0.setPosition(rightClawOpened);//Claw Open
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if (sensorDistance.getDistance(DistanceUnit.CM) <= 2) {
            if ((sensorColor.red() > sensorColor.green()) && (sensorColor.green() > sensorColor.blue())) {
                telemetry.addData("Color", "red");
                servo0.setPosition(rightClawOpened);//claw open
            } else if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.green() > sensorColor.red())) {
                telemetry.addData("Color", "blue");
                servo0.setPosition(rightClawClosed);//claw closed
                gamepad1.rumble(100);
            } else if ((sensorColor.green() > sensorColor.blue()) && (sensorColor.blue() > sensorColor.red())) {
                telemetry.addLine("OBJECT DETECTED-Wrong color");
                servo0.setPosition(rightClawOpened);//Claw Open
            } else if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.green() > sensorColor.red())) {
                telemetry.addData("Color", "yellow");
                servo0.setPosition(rightClawClosed);//claw closed
                gamepad1.rumble(100);
            } else {
                telemetry.addLine("NO OBJECT DETECTED");
                servo0.setPosition(rightClawOpened);//Claw Open
            }

        }
        else {
            servo0.setPosition(rightClawOpened);//Claw Open
        }



        telemetry.addData("Color vals, r", sensorColor.red());
        telemetry.addData("Color vals, g", sensorColor.green());
        telemetry.addData("Color vals, b", sensorColor.blue());
        telemetry.addData("Distance(cm)", sensorDistance.getDistance(DistanceUnit.CM));
    }
}


