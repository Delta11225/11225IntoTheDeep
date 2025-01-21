package org.firstinspires.ftc.teamcode.Competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp
//@Disabled
public class TeleopFINAL extends LinearOpMode {

    //LED lights
    RevBlinkinLedDriver lights;

    //DC Motors
    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor linearSlide;
    private DcMotor ascentArm;

    //Servos
    public Servo intakeArm = null;
    public Servo SpecimenClaw;
    public Servo ArmClaw;

    //Sensors
    ColorSensor sensorColor; //color sensor on intake arm
    DistanceSensor sensorDistance; //distance sensor on intake arm
    ColorSensor sensorColorSpecimenClaw; //color sensor on specimen claw
    DistanceSensor sensorDistanceSpecimenClaw;//distance sensor on specimen claw
    TouchSensor touchLinSlide; // touch sensor at bottom of linear slide to detect when it has reached bottom

    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Elapsed Time variables
    private ElapsedTime matchtime = new ElapsedTime();
    private ElapsedTime clawLastClosed = new ElapsedTime();

    //drive motor power variables
    double frontLeftV;
    double rearLeftV;
    double frontRightV;
    double rearRightV;

    //mechanum drive variables
    double forward;
    double right;
    double clockwise;
    double denominator;
    double temp;
    double side;
    double currentAngle;
    double powerMultiplier;//drive power multiplier

    //linear slide height variables
    int linearSlideTarget = 0; //setting initial target to 0
    int highBucketHeight = 3450;
    int lowBucketHeight = 1600;
    int highChamberHeight = 1800;
    int lowChamberHeight = 538;
    int highChamberReleaseHeight = 1270;
    int autoGrabLSHeight = 500;

    //ascent arm encoder locations
    int armHang = 34;
    int armHook = 8415;
    int store = 0;

    //intake arm variables
    double IntakeArmUp = .84;
    double IntakeArmHang = .90;
    double IntakeArmHoldEmpty = .59;
    double IntakeArmHoldFull = .63;
    double IntakeArmDown = .53;

    // Arm Claw Variables
    double ArmClawOpen = 0.1;
    double ArmClawClosed = 0.5;

    //specimen claw variables
    double ClawOpen = 0.4;
    double ClawClosed = 0.8;

    //boolean varibales
    boolean ArmUp = true;
    boolean slideDown = true;
    boolean slideGoingDown = false;
    boolean SpecimenclawIsOpen = true;
    boolean ClawLoaded = false;
    boolean ArmInHold = false;
    boolean AscentArmDown = true;
    boolean EndgameOverride = false;


    //String Variables
    String sampleColor = "none";

    @Override
    public void runOpMode() {
////////////////////////////////Init Phase/////////////////////////////////////////////////////////////////

    //////////////initialize IMU//////////////////////////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);

        composeTelemetry();

    ////////////initialize drive motors////////////////
        rearLeft = hardwareMap.dcMotor.get("leftRear");
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft = hardwareMap.dcMotor.get("leftFront");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight = hardwareMap.dcMotor.get("rightFront");
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        rearRight = hardwareMap.dcMotor.get("rightRear");
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

    //////////////////initialize arm claw & Sensor///////////////////////
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");//armClaw color sensor
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3"); //armClaw distance sensor
        ArmClaw = hardwareMap.get(Servo.class, "arm_claw");
        ArmClaw.setPosition(ArmClawOpen);

    //////////////////initialize linear slide///////////////////////
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setTargetPosition(linearSlideTarget);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setZeroPowerBehavior(BRAKE);
        touchLinSlide = hardwareMap.get(TouchSensor.class, "touch");

    //////////////////initialize intake arm///////////////////////
        intakeArm = hardwareMap.get(Servo.class, "intake_arm");
        intakeArm.setPosition(IntakeArmUp);



    //////////////////initialize specimen claw & sensors////////////////////////////
        SpecimenClaw = hardwareMap.get(Servo.class, "claw");
        sensorColorSpecimenClaw = hardwareMap.get(ColorSensor.class, "claw_colorV3");
        sensorDistanceSpecimenClaw = hardwareMap.get(DistanceSensor.class, "claw_colorV3");
        SpecimenClaw.setPosition(ClawOpen);


    /////////////// initialize Ascent Arm//////////////////////
        ascentArm = hardwareMap.get(DcMotor.class, "ascent_arm");
        ascentArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascentArm.setZeroPowerBehavior(BRAKE);
        //Turn on Run to Position and set initial target at store = 0
        ascentArm.setTargetPosition(store);
        ascentArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    /////////////// initialize LED lights//////////////////////
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
        //LED lights light up to signal that init phase complete


/////////////////////////////////////// End init phase/////////////////////////////////////////////////////////////

        waitForStart();
        //operations to complete 1 time at the start of teleOp (DO NOT WANT TO LOOP)
        matchtime.reset();
        clawLastClosed.reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        currentAngle = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.update();

            while (angles.firstAngle < 0 && opModeIsActive()) {

                telemetry.update();
                move();
                peripheralmove();

                currentAngle = angles.firstAngle + 360;
//                telemetry.addData("currentAngle loop 1", "%.1f", currentAngle);
            }

            while (angles.firstAngle >= 0 && opModeIsActive()) {

                telemetry.update();
                move();
                peripheralmove();

                currentAngle = angles.firstAngle;
//                telemetry.addData("currentAngle loop 2", "%.1f", currentAngle);
            }

            telemetry.addLine("null angle");
        }
    }


    public void move(){


        double theta = Math.toRadians(currentAngle);

//        telemetry.addData("CurrentAngle", currentAngle);
//        telemetry.addData("Theta", theta);

        //update to change starting orientation if needed
        forward = -gamepad1.left_stick_y; //left joystick up
        right = gamepad1.left_stick_x; //left joystick right
        clockwise = gamepad1.right_stick_x; //right joystick right (up on FTC Dashboard)

        temp = (forward * Math.cos(theta) - right * Math.sin(theta));
        side = (forward * Math.sin(theta) + right * Math.cos(theta));

        forward = temp;
        right = side;

        denominator = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(clockwise), 1);

        frontLeftV = (forward + right + clockwise) / denominator;
        rearLeftV = (forward - right + clockwise) / denominator;
        rearRightV = (forward + right - clockwise) / denominator;
        frontRightV = (forward - right - clockwise) / denominator;

        //Handle speed controls
        if (gamepad1.left_bumper) {
            powerMultiplier = .8;//FAST MODE
        } else if (gamepad1.right_bumper) {
            powerMultiplier = .3; //SLOW MODE
        } else if ((gamepad1.right_trigger >= .5) && (gamepad1.left_trigger >= .5)) {
            powerMultiplier = 1;//SUPERSPEED
        } else
            powerMultiplier = .6; //NORMAL MODE

        frontLeft.setPower(frontLeftV * powerMultiplier);
        frontRight.setPower(frontRightV * powerMultiplier);
        rearLeft.setPower(rearLeftV * powerMultiplier);
        rearRight.setPower(rearRightV * powerMultiplier);

    }

    public void peripheralmove() {
///////////////////////////////////SAMPLE COLOR DETECTION ARM CLAW///////////////////////////

        if (sensorDistance.getDistance(DistanceUnit.CM) <= 3 && ClawLoaded==true) {
            if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red())) {
                sampleColor = "blue";
            } else if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.red() > sensorColor.green())) {
                sampleColor = "red";
            } else if ((sensorColor.green() > sensorColor.red()) && (sensorColor.red() > sensorColor.blue())) {
                sampleColor = "yellow";
            }
        } else if ((sensorDistance.getDistance(DistanceUnit.CM) <= 9.5) &&
                 ArmUp == false && ClawLoaded==false) {
            if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red())) {
                sampleColor = "blue-detected";
            } else if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.red() > sensorColor.green())) {
                sampleColor = "red-detected";
            } else if ((sensorColor.green() > sensorColor.red()) && (sensorColor.red() > sensorColor.blue()) && (sensorColor.green() > 100)) {
                sampleColor = "yellow-detected";
            }
        }
        else {
                sampleColor = "none";
            }

        ////////////////MATCH TIMER LEDs///////////////////////////
        if ((matchtime.seconds() > 85) && (matchtime.seconds() < 90)) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
            EndgameOverride = true;
        } else if ((matchtime.seconds() > 100) && matchtime.seconds() < 120) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
            EndgameOverride = true;
        } else if ((matchtime.seconds() > 120)) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
            EndgameOverride = true;
        } else
            EndgameOverride = false;

//////////////////////////////SAMPLE DETECTION LED SIGNALS//////////////////////////////////////

        if (EndgameOverride == false) {
            if (sampleColor == "blue") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (sampleColor == "red") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (sampleColor == "yellow") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            } else if (sampleColor == "blue-detected") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            } else if (sampleColor == "red-detected") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            } else if (sampleColor == "yellow-detected") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            } else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
        }


        //////////////GAMEPAD 1//////////////

/////////////////////////////////////Ascent Arm Auto//////////////////////////////////////////////////////
        if (gamepad1.dpad_down & gamepad1.a) {
            ascentArm.setTargetPosition(store);
            ascentArm.setPower(0.5);
            AscentArmDown = true;
        }

        if (gamepad1.dpad_left & gamepad1.b) {
            ascentArm.setPower(1);
            ascentArm.setTargetPosition(armHook);
            ArmClaw.setPosition(ArmClawClosed);
            SpecimenClaw.setPosition(ClawClosed);
            AscentArmDown = false;
        }

        if (gamepad1.dpad_up & gamepad1.y) {
            ascentArm.setPower(1);
            ascentArm.setTargetPosition(armHang);
            //close specimen claw for safe hanging
            SpecimenClaw.setPosition(ClawClosed);
            ArmClaw.setPosition(ArmClawClosed);
            intakeArm.setPosition(IntakeArmHang);

            AscentArmDown = false;
        }
        if (matchtime.seconds()<90 && matchtime.seconds()>91)
            gamepad2.rumble(500);


                                //////////////GAMEPAD 2//////////////

//////////////////////////MANUAL ARM CLAW CONTROLS///////////////////////////////////////////
        if (gamepad2.right_bumper) {
        ArmClaw.setPosition(ArmClawClosed);
        ClawLoaded = true;
        }
        else if (gamepad2.left_bumper) {
        ArmClaw.setPosition(ArmClawOpen);
        ClawLoaded = false;
        }

//////////////////////////////////////////linear slide///////////////////////

        if (gamepad2.y && ArmUp == true) {
            slideDown = false;
            linearSlideTarget = highBucketHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
            SpecimenClaw.setPosition(ClawClosed);
        }
        if (gamepad2.x && ArmUp == true) {
            slideDown = false;
            linearSlideTarget = lowBucketHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
            SpecimenClaw.setPosition(ClawClosed);
        }
        // bring slide to ground
        if (gamepad2.left_stick_y > 0.5 && gamepad2.right_stick_y > 0.5) {
            slideGoingDown = true;
            linearSlideTarget = 0;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
            SpecimenClaw.setPosition(ClawOpen);
            SpecimenclawIsOpen = true;

        }
        if (touchLinSlide.isPressed() == false && linearSlideTarget <= 0 && slideGoingDown == true) {
            slideGoingDown = true;
            linearSlideTarget = -5000;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(0.9);

        }
        //slide has reached ground position
        if (touchLinSlide.isPressed() == true && slideGoingDown == true) {
            slideDown=true;
            slideGoingDown = false;
            linearSlide.setTargetPosition(linearSlide.getCurrentPosition());
            linearSlide.setPower(0);
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlide.setTargetPosition(0);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.b && ArmUp == true) {
            slideDown = false;
            linearSlideTarget = highChamberHeight;//high chamber height
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        if (gamepad2.a && ArmUp == true) {
            slideDown = false;
            linearSlideTarget = highChamberReleaseHeight;//high chamber release height
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }

////////////////////////////////////////intake arm///////////////////////
        // brings arm to score/up position
        if (gamepad2.dpad_up) {
            intakeArm.setPosition(IntakeArmUp);
            ArmUp = true;
            ArmInHold = false;
        }
        // brings arm to down/collect position
        if (gamepad2.dpad_down && slideDown == true) {
            if (ArmUp == true){
                ArmClaw.setPosition(ArmClawOpen);
                ClawLoaded = false;
            }
            if (ArmInHold == true) {
                ArmClaw.setPosition(ArmClawOpen);
                ClawLoaded = false;
            }
            intakeArm.setPosition(IntakeArmDown);
            ArmUp = false;
            ArmInHold = false;
        }
        // brings arm to hold position
        if (gamepad2.dpad_left && slideDown == true) {
            if (ArmUp == true){
                ArmClaw.setPosition(ArmClawOpen);
                ClawLoaded = false;
            }
            ArmInHold = true;
            ArmUp = false;
        }

        if (ArmInHold == true && ClawLoaded == true) {
            intakeArm.setPosition(IntakeArmHoldFull);
            ArmUp = false;
        }
        if (ArmInHold == true && ClawLoaded == false) {
            intakeArm.setPosition(IntakeArmHoldEmpty);
            ArmUp = false;
        }

////////////////////////////////////////manual speciment claw controls///////////////////////

        if (gamepad2.left_trigger > 0.5) {
            SpecimenClaw.setPosition(ClawOpen);
            SpecimenclawIsOpen = true;
        }

        if (gamepad2.right_trigger > 0.5) {
            SpecimenClaw.setPosition(ClawClosed);
            SpecimenclawIsOpen = false;
        }

////////////////////////////////////////SPECIMEN CLAW AUTOGRAB///////////////////////
        //make sure to raise linear slide above wall after grabbing
        if (sensorDistanceSpecimenClaw.getDistance(DistanceUnit.CM) <= 5 && SpecimenclawIsOpen==true && slideDown == true && clawLastClosed.seconds() > 1 && AscentArmDown == true && ArmUp == true) {
            gamepad2.rumble(500);
            SpecimenClaw.setPosition(ClawClosed);//Claw Closed
            clawLastClosed.reset();
            SpecimenclawIsOpen=false;
        }
        if(SpecimenclawIsOpen == false && clawLastClosed.seconds()>0.25 && clawLastClosed.seconds()<1){
            slideDown = false;
            linearSlideTarget = autoGrabLSHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(0.6);
        }
        telemetry.addData(" specimen claw open", SpecimenclawIsOpen);
        telemetry.addData("Distance(specimen claw)", sensorDistanceSpecimenClaw.getDistance(DistanceUnit.CM));
        telemetry.addData("Sample claw closed", ClawLoaded);
        telemetry.addData("Distance(sample claw)", sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Sample Claw Color vals, r", sensorColor.red());
        telemetry.addData("Sample Claw Color vals, g", sensorColor.green());
        telemetry.addData("Sample Claw Color vals, b", sensorColor.blue());
        telemetry.addData("Sample Color",sampleColor);
    //////////////////IMU Reset///////////////////////////
    if(gamepad1.right_trigger > 0.6 && gamepad1.left_trigger > 0.6) {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    telemetry.addData("current angle", currentAngle);
    }


/////end of peripheral move////////

    /*-----------------------------------//
    * DO NOT WRITE CODE BELOW THIS LINE  *
    * -----------------------------------*/
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel*gravity.xAccel
//                                        + gravity.yAccel*gravity.yAccel
//                                        + gravity.zAccel*gravity.zAccel));
//                    }
//                });

        // telemetry.addData("currentAngle", "%.1f", currentAngle);
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}