package org.firstinspires.ftc.teamcode.lessons;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IfElseTestJTH extends OpMode {
    public DcMotor motor1 = null;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor_1");
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_x>=0.5) {
            motor1.setPower(1);
        }
        else if(gamepad1.left_stick_x<-0.5) {
            motor1.setPower(-0.5);
        }
        else{
            motor1.setPower(0);
        }

        telemetry.addData("a button", gamepad1.a);
    }

}
