package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp
public class LinearSlideTestPierson extends OpMode {
    private DcMotor linearSlide;


    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        linearSlide.setDirection(DcMotor.Direction.REVERSE); //reverses all values
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && (linearSlide.getCurrentPosition()<22100)) {
            linearSlide.setPower(1);
        }
        else if (gamepad1.dpad_down && (linearSlide.getCurrentPosition()>0)) {
            linearSlide.setPower(-1);
        }
        else {
            linearSlide.setPower(0);
        }

        telemetry.addData("encoder", linearSlide.getCurrentPosition());
    }


}