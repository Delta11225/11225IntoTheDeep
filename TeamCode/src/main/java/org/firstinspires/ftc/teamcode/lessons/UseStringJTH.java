package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class UseStringJTH extends OpMode {
    @Override
    public void init() {
        String myName ="Holden Boles";

        telemetry.addData("Hello", myName);
        telemetry.addLine("has to be funny");
    }
    @Override
    public void loop() {

    }
}
