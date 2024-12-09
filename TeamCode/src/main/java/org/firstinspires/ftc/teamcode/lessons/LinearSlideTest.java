package org.firstinspires.ftc.teamcode.lessons;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
@TeleOp
public class LinearSlideTest extends OpMode {

    private DcMotor linearSlide;
    double maxHeightLS;//22104
    double minHeightLS;
    TouchSensor touch;


    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(BRAKE);
        touch = hardwareMap.get(TouchSensor.class, "touch");

    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            linearSlide.setPower(0.3);

        }
        else if (gamepad1.dpad_down && touch.isPressed()==false) {
            linearSlide.setPower(-1);
        }

        else {
            linearSlide.setPower(0.0);
        }





        telemetry.addData("encoder", linearSlide.getCurrentPosition());
        telemetry.addData("touch state",touch);
        telemetry.update();
    }

}


