package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp
public class NewArmClawTest extends OpMode {

    //LED lights
    RevBlinkinLedDriver lights;

    Servo IntakeArm;
    Servo ArmClaw;
    ColorSensor sensorColor; //color sensor on intake arm
    DistanceSensor sensorDistance; //distance sensor on intake arm

    private ElapsedTime matchtime = new ElapsedTime();

    // Arm Claw Variables
    double ArmClawOpen = 0;
    double ArmClawClosed = 0.5;

    //claw arm varis
    double IntakeArmDown = .5;
    double IntakeArmHold = .55;
    double IntakeArmUp = .84;

    //String Variables
    String sampleColor = "none";

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        //////////////////initialize arm claw & Sensor///////////////////////
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");//armClaw color sensor
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3"); //armClaw distance sensor
        ArmClaw = hardwareMap.get(Servo.class, "arm_claw");
        IntakeArm = hardwareMap.get(Servo.class, "intake_arm");
        ArmClaw.setPosition(ArmClawOpen);

        /////////////// initialize LED lights//////////////////////
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        //LED lights light up to signal that init phase complete
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);

        //operations to complete 1 time at the start of teleOp (DO NOT WANT TO LOOP)
        matchtime.reset();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        ///////////////////////////////////SAMPLE COLOR DETECTION ARM CLAW///////////////////////////

        if (sensorDistance.getDistance(DistanceUnit.CM) <= 2) {
            if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red())) {
                sampleColor = "blue";
            } else if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.red() > sensorColor.green())) {
                sampleColor = "red";
            } else if ((sensorColor.green() > sensorColor.red()) && (sensorColor.red() > sensorColor.blue())) {
                sampleColor = "yellow";
            }
        } else if ((sensorDistance.getDistance(DistanceUnit.CM) <= 10) && (sensorDistance.getDistance(DistanceUnit.CM) >2)) {
            if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red())) {
                sampleColor = "blue-detected";
            } else if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.red() > sensorColor.green())) {
                sampleColor = "red-detected";
            } else if ((sensorColor.green() > sensorColor.red()) && (sensorColor.red() > sensorColor.blue())) {
                sampleColor = "yellow-detected";
            }
                else {
                    sampleColor = "none";
                }
            }
//////////////////////////////SAMPLE DETECTION LED SIGNALS//////////////////////////////////////

            if (sampleColor == "blue") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            } else if (sampleColor == "red") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            } else if (sampleColor == "yellow") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            } else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            ////////////////match time leds///////////////////////////


            if ((matchtime.seconds() > 100) && matchtime.seconds() < 120) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);

            } else if ((matchtime.seconds() > 120)) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
            }

//////////////////////////////New Arm Claw//////////////////////////////////////
            if ((sampleColor == "blue") || (sampleColor == "yellow")) {
                ArmClaw.setPosition(ArmClawClosed);
            } else {
                ArmClaw.setPosition(ArmClawOpen);
            }
            if(gamepad1.y) {
                IntakeArm.setPosition(IntakeArmUp);
            }
            if(gamepad1.b) {
                IntakeArm.setPosition(IntakeArmHold);}

            if(gamepad1.a) {
                    IntakeArm.setPosition(IntakeArmHold);
            }

            if (gamepad1.left_trigger < -0.5) {
                ArmClaw.setPosition(ArmClawOpen);
            } else if (gamepad1.right_trigger >= 0.5) {
                ArmClaw.setPosition(ArmClawClosed);
            }

            telemetry.addData("Color vals, r", sensorColor.red());
            telemetry.addData("Color vals, g", sensorColor.green());
            telemetry.addData("Color vals, b", sensorColor.blue());
            telemetry.addData("Distance(cm)", sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Sample color detected", sampleColor);
        }
    }


