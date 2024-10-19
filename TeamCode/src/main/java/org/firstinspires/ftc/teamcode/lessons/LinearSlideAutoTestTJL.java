package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class LinearSlideAutoTestTJL extends OpMode {

    private DcMotor linearSlide;
    private int linearSlideTarget = 0;
    private int linearSlideZero = 0;
    boolean sliderunning = false;
    TouchSensor touch;

    @Override
    public void init(){
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setTargetPosition(linearSlideTarget);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        touch = hardwareMap.get(TouchSensor.class, "touch");
    }

    @Override
    public void loop(){
        //set linear slide target value
        if (gamepad1.dpad_up) {
            linearSlideTarget = 22000;
            linearSlide.setTargetPosition(linearSlideZero + linearSlideTarget);
            linearSlide.setPower(0.3);
        }
        else if (gamepad1.dpad_down && touch.isPressed()==false) {
            linearSlideTarget = 0;
            linearSlide.setTargetPosition(linearSlideZero + linearSlideTarget);
            linearSlide.setPower(0.3);
        }


        if (touch.isPressed() && linearSlideZero+linearSlideTarget<20) {
            linearSlide.setPower(0);
            linearSlideZero = linearSlide.getCurrentPosition();

        }





        telemetry.addData("encoder", linearSlide.getCurrentPosition());
    }

}
