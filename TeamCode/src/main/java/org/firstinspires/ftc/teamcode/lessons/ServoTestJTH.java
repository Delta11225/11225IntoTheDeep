package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class ServoTestJTH extends OpMode {
    public Servo servo1 = null;
    public void init() {
    servo1 = hardwareMap.get(Servo.class,"intake_arm");
    }
    double servoPosition = 0.84;

    public void loop(){
        if (gamepad1.x){
            //servo1.setPosition(0);
            servoPosition = 0.5;
        }
        else if (gamepad1.y) {
            servoPosition = 0.6;
        }
        else if (gamepad1.b){

            servoPosition = 0.84;
        }
        servo1.setPosition(servoPosition);

        telemetry.addData("position",servoPosition);
        telemetry.addData("servo 1",servo1.getPosition());
    }
}


