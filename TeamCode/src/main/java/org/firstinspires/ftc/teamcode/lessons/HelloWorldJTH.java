package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous()
public class HelloWorldJTH extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","James, Tonya, and Holden :)");
    }

    @Override
    public void loop() {

    }
}
