package org.firstinspires.ftc.teamcode.testing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class AscentArmTest extends OpMode {

    private DcMotor ascentArm;
    double maxHeightLS;//22104
    double minHeightLS;
    TouchSensor touch;


    @Override
    public void init() {
        ascentArm = hardwareMap.get(DcMotor.class, "ascent_arm");
        ascentArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ascentArm.setDirection(DcMotor.Direction.REVERSE);
        ascentArm.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            ascentArm.setPower(1);

        }
        else if (gamepad1.dpad_down) {
            ascentArm.setPower(-1);
        }

        else {
            ascentArm.setPower(0.0);
        }


        telemetry.addData("encoder", ascentArm.getCurrentPosition());
        telemetry.update();
    }

}


