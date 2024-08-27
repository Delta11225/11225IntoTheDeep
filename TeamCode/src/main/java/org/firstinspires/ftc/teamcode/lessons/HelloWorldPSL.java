package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class HelloWorldPSL extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","Pierson, Lillian, Sophia");
    }

    @Override
    public void loop() {

    }
}
