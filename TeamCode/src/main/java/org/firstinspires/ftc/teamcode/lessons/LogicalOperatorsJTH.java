package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LogicalOperatorsJTH extends OpMode {
    public DcMotor motor1 = null;
    @Override
    public void init()  {
        motor1 = hardwareMap.get(DcMotor.class, "motor_1");
    }
    @Override
    public void loop() {
        if (gamepad1.a && gamepad1.b){
            motor1.setPower(1);
        }
        else if (gamepad1.x && gamepad1.y){
            motor1.setPower(-1);
        } else {
            motor1.setPower(0);
        }
    }
}
