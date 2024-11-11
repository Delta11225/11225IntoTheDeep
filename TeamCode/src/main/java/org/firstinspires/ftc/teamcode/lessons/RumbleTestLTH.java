package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class RumbleTestLTH extends OpMode {
    DistanceSensor sensorDistance;
    public ElapsedTime matchtime = new ElapsedTime();
@Override
    public void init() {
    sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3");}
@Override
public void start() {
    matchtime.reset();
}
@Override
    public void loop() {
    if(sensorDistance.getDistance(DistanceUnit.CM) <= 3){
        gamepad1.rumble(100);
    }
    else {

    }
}
}
