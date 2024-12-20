package org.firstinspires.ftc.teamcode.Competition;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

//@Disabled
@TeleOp
public class AscentArmReset extends OpMode {

    private DcMotor ascentArm;
    double maxHeightLS;//22104
    double minHeightLS;
    TouchSensor touch;
    public Servo ArmClaw;
    double ArmClawClosed = 0.5;

    @Override
    public void init() {
        ascentArm = hardwareMap.get(DcMotor.class, "ascent_arm");
        ascentArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ascentArm.setDirection(DcMotor.Direction.REVERSE);
        ascentArm.setZeroPowerBehavior(BRAKE);
        ArmClaw = hardwareMap.get(Servo.class, "arm_claw");

        ArmClaw.setPosition(ArmClawClosed);
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            ascentArm.setPower(0.5);

        }
        else if (gamepad1.dpad_down) {
            ascentArm.setPower(-0.5);
        }

        else {
            ascentArm.setPower(0.0);
        }


        telemetry.addData("encoder", ascentArm.getCurrentPosition());
        telemetry.update();
    }

}


