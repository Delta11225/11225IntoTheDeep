package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class IntakeAutoGrabChallengeBlueLTH extends OpMode{

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    public DcMotor motor1 = null;
    boolean intakeRunning = true;
    String sampleColor = "none";

    Double powerIn = 0.5;
    Double powerOut = -0.5;
    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor_1");
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3");

    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red()) //check color blue
                && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                && (intakeRunning == true))//claw is open
        {
            sampleColor = "blue";
            motor1.setPower(powerIn);//Taking in sample
            gamepad1.rumble(500);
            intakeRunning = false;

        }
        if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.green() > sensorColor.red())//check color yellow
                && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                && (clawOpen == true))//claw is open
        {
            sampleColor = "yellow";
            servo0.setPosition(rightClawClosed);//claw closed
            servo1.setPosition(leftClawClosed);//Claw Closed
            gamepad1.rumble(100);
            clawOpen = false;
        }


        if (gamepad1.right_bumper){
            servo0.setPosition(rightClawOpened);//Claw Open
            servo1.setPosition(leftClawOpened);//Claw Open
            clawOpen = true;
            sampleColor = "none";
        }

        telemetry.addData("Color vals, r", sensorColor.red());
        telemetry.addData("Color vals, g", sensorColor.green());
        telemetry.addData("Color vals, b", sensorColor.blue());
        telemetry.addData("Distance(cm)", sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Color Detected",sampleColor);
        telemetry.addData("power",powerIn);
        telemetry.addData("power",powerOut);
        telemetry.addData("motor1",motor1.getPower());
    }

}

