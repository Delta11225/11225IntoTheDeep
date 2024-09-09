package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTestJTH extends OpMode {
    public Servo servo1 = null;
    public void init() {
    servo1 = hardwareMap.get(Servo.class,"servo_1");
    }
    double servoPosition = 0;

    public void loop(){
        if (gamepad1.x){
            //servo1.setPosition(0);
            servoPosition = 0;
        }
        else if (gamepad1.y) {
            servoPosition = 0.5;
        }
        else if (gamepad1.b){

            servoPosition = 1;
        }
        servo1.setPosition(servoPosition);

        telemetry.addData("position",servoPosition);
        telemetry.addData("servo 1",servo1.getPosition());
    }
}


