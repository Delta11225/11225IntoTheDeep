package org.firstinspires.ftc.teamcode.lessons;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp
public class LinearSlideAutoTestChallengeTL extends OpMode {

    private DcMotor linearSlide;
    private int linearSlideTarget = 0;
    private int linearSlideZero = 0;
    boolean sliderunning = false;
    TouchSensor touch;
    int highBasketHeight = 3600;
    int lowBasketHeight = 1675;
    int highChamberHeight = 1875;
    int lowChamberHeight = 538;

    public void init() {
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setTargetPosition(linearSlideTarget);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setZeroPowerBehavior(BRAKE);
        touch = hardwareMap.get(TouchSensor.class, "touch");
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            linearSlideTarget = highBasketHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        if (gamepad1.x) {
            linearSlideTarget = lowBasketHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        if (gamepad1.a) {
            linearSlideTarget = 0;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(0.5);
        }
        if (touch.isPressed() == true) {
            linearSlide.setTargetPosition(linearSlide.getCurrentPosition());
            linearSlide.setPower(0);
        }
        if (gamepad1.dpad_up) {
            linearSlideTarget = 1875;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        if (gamepad1.dpad_down) {
            linearSlideTarget = 538;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        telemetry.addData("encoder", linearSlide.getCurrentPosition());
    }
}