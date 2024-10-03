package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LinearSlideTestTHL extends OpMode {

    private DcMotor linearSlide;

    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            linearSlide.setPower(0.5);
        }

        else if (gamepad1.dpad_down){
            linearSlide.setPower(-0.5);
        }

        else {
            linearSlide.setPower(0.0);
        }

        telemetry.addData("encoder", linearSlide.getCurrentPosition());
        telemetry.update();
    }

}


