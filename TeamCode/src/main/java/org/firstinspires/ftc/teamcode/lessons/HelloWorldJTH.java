package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HelloWorldJTH extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","World :)");
    }

    @Override
    public void loop() {

    }
}
