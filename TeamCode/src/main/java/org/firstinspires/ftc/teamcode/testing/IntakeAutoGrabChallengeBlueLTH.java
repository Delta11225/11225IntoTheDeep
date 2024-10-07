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
    public DcMotor intake = null;
    boolean intakeRunning = true;
    String sampleColor = "none";

    Double powerIn = 0.5;
    Double powerOut = -0.5;
    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake_1");
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3");

    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if (gamepad1.right_bumper){

            if (sensorDistance.getDistance(DistanceUnit.CM) >= 2){
                intakeRunning = true;
                intake.setPower(powerIn);
            }

            else if ((sampleColor == "blue") //check color blue
                    && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                    && (intakeRunning == true))//claw is open
            {
            sampleColor = "blue";
            intake.setPower(0);//Taking in sample
            gamepad1.rumble(500);
            intakeRunning = false;

            }

            else if ((sampleColor == "red")
                && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                && (intakeRunning == true))//intake is running
            {
                sampleColor = "red";
                intake.setPower(powerOut);//intake is running counterclockwise
                gamepad1.rumble(100);
                intakeRunning = true;
            }
            else if ((sampleColor == "yellow")//check color yellow
                && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                && (intakeRunning == true))//intake is running
            {
            sampleColor = "yellow";
            intake.setPower(0);//intake is running
            gamepad1.rumble(100);
            intakeRunning = false;
            }

        }

        else{
            intakeRunning = false;
            intake.setPower(0);
        }



        telemetry.addData("Color vals, r", sensorColor.red());
        telemetry.addData("Color vals, g", sensorColor.green());
        telemetry.addData("Color vals, b", sensorColor.blue());
        telemetry.addData("Distance(cm)", sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Color Detected",sampleColor);
        telemetry.addData("intake power",intake.getPower());


        ///////////////////SAMPLE COLOR DETECTION///////////////////////////

        if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red())){
            sampleColor = "blue";
        }
        if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.red() > sensorColor.green())){
            sampleColor = "red";
        }

        if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.green() > sensorColor.red())){
            sampleColor = "yellow";
        }

}
}
