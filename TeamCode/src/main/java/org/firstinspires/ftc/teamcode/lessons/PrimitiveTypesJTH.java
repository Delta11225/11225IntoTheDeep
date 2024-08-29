package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class PrimitiveTypesJTH extends OpMode {
    @Override
    public void init() {
        int teamNumber = 11225;
        double motorSpeed = 0.5;
        boolean touchSensorPressed = true;
        int grade = 100;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);
        telemetry.addData("Grade",grade);
    }

    @Override
    public void loop() {

    }
}





