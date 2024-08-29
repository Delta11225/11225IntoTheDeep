package org.firstinspires.ftc.teamcode.lessons;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DCmotortestJTH extends OpMode {
    public DcMotor motor1 = null;

@Override
public void init() {
    motor1 = hardwareMap.get(DcMotor.class, "motor_1");
}
@Override
public void loop(){

    double power=1.0;

    motor1.setPower(power);

    telemetry.addData("power",power);

}


}
