package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorDistanceSensorLT extends OpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3");
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if (sensorDistance.getDistance(DistanceUnit.CM) <= 5) {
            if ( (sensorColor.red() > sensorColor.green()) && (sensorColor.red() > sensorColor.blue())) {
                telemetry.addData("Color", "red");
            } else if ( (sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red())) {
                telemetry.addData("Color", "blue");
            } else if ((sensorColor.blue() < 2000) || (sensorColor.red() < 2000)) {
                telemetry.addLine("OBJECT DETECTED-Wrong color");
            } else {
                telemetry.addLine("NO OBJECT DETECTED");
            }

        }
        telemetry.addData("Color vals, r", sensorColor.red());
        telemetry.addData("Color vals, g", sensorColor.green());
        telemetry.addData("Color vals, b", sensorColor.blue());
        telemetry.addData("Distance(cm)", sensorDistance.getDistance(DistanceUnit.CM));
    }
}


