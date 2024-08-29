package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DCMotorTestPSL extends OpMode {
    //Declare OpMode members
    public DcMotor motor1 = null;
    //Code to run ONCE when the driver hits INIT
    @Override
    public void init(){
        //Define and Initialize Motors
        motor1 = hardwareMap.get(DcMotor.class, "motor_1");
    }
    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop(){
    }
    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start(){
    }
    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop(){

        double power = -gamepad1.right_stick_x;

        motor1.setPower(power);

        telemetry.addData("power",power);
        telemetry.addData("motor1",motor1.getPower());

    }
    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop(){
    }
}
