package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp
public class LinearSlideAutoTestPierson extends OpMode {
    private DcMotor linearSlide;
    private int linearSlideTarget = 0;


    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlide.setTargetPosition(linearSlideTarget);
        linearSlide.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        linearSlide.setDirection(DcMotor.Direction.REVERSE); //reverses all values
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            linearSlideTarget = 20000;
        }

        if (gamepad1.b){
            linearSlideTarget = 11000;
        }
        if (gamepad1.a) {
            linearSlideTarget = 0;
        }


        linearSlide.setTargetPosition(linearSlideTarget);
        linearSlide.setPower(1);
        telemetry.addData("encoder", linearSlide.getCurrentPosition());
    }

}


