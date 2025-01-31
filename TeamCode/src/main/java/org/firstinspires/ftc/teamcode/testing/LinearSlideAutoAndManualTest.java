package org.firstinspires.ftc.teamcode.testing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp
public class LinearSlideAutoAndManualTest extends OpMode {

    private DcMotor linearSlide;
    private int linearSlideTarget = 0;
    private int linearSlideZero = 0;
    boolean sliderunning = false;
    TouchSensor touch;
    int highBucketHeight = 3600;
    int lowBucketHeight = 1675;
    int highChamberHeight = 1875;
    int lowChamberHeight = 538;
    int highChamberReleaseHeight = 1250;

    //booleans to handle slide state
    boolean slideDown = true;
    boolean slideGoingDown = false;

    // How much to change linear linear target when lift or lower buttons are pressed
    public final static int upEncoderStep = 30;
    public final static int downEncoderStep = 30;

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

        //auto slide to high bucket
        if (gamepad2.y) {
            slideDown = false;
            linearSlideTarget = highBucketHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }

        //auto slide to low bucket
        if (gamepad2.x) {
            slideDown = false;
            linearSlideTarget = lowBucketHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }

        // auto bring slide to ground
        if (gamepad2.left_stick_y > 0.5 && gamepad2.right_stick_y > 0.5) {
            slideGoingDown = true;
            linearSlideTarget = 0;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(0.8);
        }

        //slide has reached ground position
        if (touch.isPressed() == true && slideGoingDown == true) {
            slideDown=true;
            slideGoingDown = false;
            linearSlide.setTargetPosition(linearSlide.getCurrentPosition());
            linearSlide.setPower(0);
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlide.setTargetPosition(0);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //auto slide to high chamber
        if (gamepad2.b) {
            slideDown = false;
            linearSlideTarget = highChamberHeight;//high chamber height
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        //auto slide to high chamber release height
        if (gamepad2.a) {
            slideDown = false;
            linearSlideTarget = highChamberReleaseHeight;//high chamber release height
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }

        //manual DOWN control for slide as long as touch is not pressed
        if (gamepad2.dpad_down && touch.isPressed()==false && slideDown==false){
            slideGoingDown = true;
            linearSlideTarget = linearSlideTarget - 30;//incrimental decrease in target to bring down
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        //manual UP control for slide as long as touch is not pressed
        if (gamepad2.dpad_up){
            slideGoingDown = false;
            linearSlideTarget = linearSlideTarget + 30;//incrimental decrease in target to bring down
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }

        telemetry.addData("encoder", linearSlide.getCurrentPosition());
    }
}