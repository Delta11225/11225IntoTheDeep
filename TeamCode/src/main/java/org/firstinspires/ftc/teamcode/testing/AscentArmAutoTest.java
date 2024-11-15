package org.firstinspires.ftc.teamcode.testing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class AscentArmAutoTest extends OpMode {
    private DcMotor ascentArm;
    int armHang = 234;
    int armHook = 8515;
    int store = 0;


    @Override
    public void init() {
        ascentArm = hardwareMap.get(DcMotor.class, "ascent_arm");
        ascentArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascentArm.setZeroPowerBehavior(BRAKE);
        //Turn on Run to Position and set initial target at store = 0
        ascentArm.setTargetPosition(store);
        ascentArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    @Override
    public void loop(){

        if (gamepad1.dpad_down & gamepad1.a){
            ascentArm.setTargetPosition(store);
            ascentArm.setPower(0.5);


        }
        if (gamepad1.dpad_left & gamepad1.b) {
            ascentArm.setPower(1);
            ascentArm.setTargetPosition(armHook);
           ;
        }

        if (gamepad1.dpad_up & gamepad1.y){
            ascentArm.setPower(1);
            ascentArm.setTargetPosition(armHang);

        }


        telemetry.addData("encoder", ascentArm.getCurrentPosition());
        telemetry.update();
    }

}


