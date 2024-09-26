package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ClawAutoGrabBlueRumbleL extends OpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    public Servo servo0 = null;
    public Servo servo1 = null;

    double leftClawClosed = 0.8;
    double leftClawOpened = 0.4;
    double rightClawClosed = 0.1;
    double rightClawOpened = 0.4;

    boolean clawOpen = true;
    String sampleColor = "none";


    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        servo0 = hardwareMap.get(Servo.class, "servo_0");
        servo1 = hardwareMap.get(Servo.class, "servo_1");
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3");
        servo0.setPosition(rightClawOpened);//Claw Open
        servo1.setPosition(leftClawOpened);//Claw Open
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
             if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red())//check color blue
                 && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                 && (clawOpen == true))//claw is open
             {
                sampleColor = "blue";
                servo0.setPosition(rightClawClosed);//claw closed
                servo1.setPosition(leftClawClosed);//Claw Closed
                gamepad1.rumble(500);
                clawOpen = false;
            }
             if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.green() > sensorColor.red())
                     && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                     && (clawOpen == true))//claw is open
             {
                sampleColor = "yellow";
                servo0.setPosition(rightClawClosed);//claw closed
                servo1.setPosition(leftClawClosed);//Claw Closed
                gamepad1.rumble(500);
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
    }
}


