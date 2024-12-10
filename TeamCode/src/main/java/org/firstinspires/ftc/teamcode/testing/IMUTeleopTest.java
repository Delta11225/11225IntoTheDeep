package org.firstinspires.ftc.teamcode.testing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.utility.Constants;
import org.firstinspires.ftc.teamcode.utility.ControlConfig;
import org.firstinspires.ftc.teamcode.utility.HardwareCC;
import org.firstinspires.ftc.teamcode.utility.HardwareITD;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.utility.Constants.armCollectPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armHoldPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armScoringPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.armTrussHeight;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampClosedPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.clampOpenPosition;
import static org.firstinspires.ftc.teamcode.utility.Constants.droneHold;
import static org.firstinspires.ftc.teamcode.utility.Constants.droneLaunch;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutomatedDeployHigh;
import static org.firstinspires.ftc.teamcode.utility.Constants.linearSlideAutomatedDeployLow;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorHookHeightLeft;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorHookHeightRight;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorLiftHeightLeft;
import static org.firstinspires.ftc.teamcode.utility.Constants.scissorLiftHeightRight;

@TeleOp
//@Disabled
public class IMUTeleopTest extends LinearOpMode {

    //HardwareITD robot;

    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public Servo claw;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime lastSlideDown = new ElapsedTime();
    private final ElapsedTime lastGrab = new ElapsedTime();


    private ElapsedTime clawLastClosed = new ElapsedTime();

    double frontLeftV;
    double rearLeftV;
    double frontRightV;
    double rearRightV;

    double forward;
    double right;
    double clockwise;

    double powerMultiplier = 0.7;
    double deadZone = Math.abs(0.2);

    int linearSlideZeroOffset = 0;

    public CRServo intake = null;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    ColorSensor sensorColorClaw;
    DistanceSensor sensorDistanceClaw;
    String sampleColor = "none";
    TouchSensor touch;

    boolean intakeRunning = true;
    boolean intakeUp = true;
    boolean sliderunning = false;

    private DcMotor linearSlide;
    private int linearSlideTarget = 0;
    private int linearSlideZero = 0;


    int highBucketHeight = 3600;
    int lowBucketHeight = 1600;
    int highChamberHeight = 1875;
    int lowChamberHeight = 538;
    int highChamberReleaseHeight = 1250;

    //from ServoTestJTH
    public Servo intakeArm = null;

    double servoPosition = 0.84;

    double IntakeArmUp = .86;
    double IntakeArmHold = .6;
    double IntakeArmDown = .52;

    double ClawOpen = 0.4;
    double ClawClosed = 0.8;
    double leftClawClosed = 0.8;
    double leftClawOpened = 0.4;
    double rightClawClosed = 0.1;
    double rightClawOpened = 0.4;

    double powerIn = 1.0;
    double powerOut = -1.0;

    double denominator;
    double temp;
    double side;

    double IMUAngle;
    double currentAngle;

    boolean slideDown = true;
    boolean slowMode = false;
    boolean slideGoingDown = false;
    boolean armIsScoring = false;
    boolean clawIsOpen = true;

    //from AscentArmAutoTest
    private DcMotor ascentArm;
    int armHang = 234;
    int armHook = 8515;
    int store = 0;


    @Override
    public void runOpMode() {

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

        //robot = new HardwareITD(hardwareMap);
        rearLeft = hardwareMap.dcMotor.get("leftRear");
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft = hardwareMap.dcMotor.get("leftFront");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight = hardwareMap.dcMotor.get("rightFront");
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        rearRight = hardwareMap.dcMotor.get("rightRear");
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //initialize drive motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

        //////////////////intake & auto grab///////////////////////
        intake = hardwareMap.get(CRServo.class, "intake");
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");//intake color sensor
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3"); //intake distance sensor


        //////////////////linear slide///////////////////////
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setTargetPosition(linearSlideTarget);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setZeroPowerBehavior(BRAKE);
        touch = hardwareMap.get(TouchSensor.class, "touch");

        //////////////////intake arm///////////////////////
        intakeArm = hardwareMap.get(Servo.class, "intake_arm");
        intakeArm.setPosition(IntakeArmUp);

        //////////////////claw////////////////////////////
        claw = hardwareMap.get(Servo.class, "claw");
        sensorColorClaw = hardwareMap.get(ColorSensor.class, "claw_colorV3");
        sensorDistanceClaw = hardwareMap.get(DistanceSensor.class, "claw_colorV3");
        claw.setPosition(ClawOpen);


        ///////////////Ascent Arm//////////////////////
        ascentArm = hardwareMap.get(DcMotor.class, "ascent_arm");
        ascentArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascentArm.setZeroPowerBehavior(BRAKE);
        //Turn on Run to Position and set initial target at store = 0
        ascentArm.setTargetPosition(store);
        ascentArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        // End init phase
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        currentAngle = 0;
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            ControlConfig.update(gamepad1, gamepad2);

            telemetry.update();

            while (angles.firstAngle < 0 && opModeIsActive()) {
                ControlConfig.update(gamepad1, gamepad2);

                telemetry.update();
                move();
                peripheralmove();


                currentAngle = angles.firstAngle + 360;
                telemetry.addData("currentAngle loop 1", "%.1f", currentAngle);
            }

            while (angles.firstAngle >= 0 && opModeIsActive()) {
                ControlConfig.update(gamepad1, gamepad2);

                telemetry.update();
                move();
                peripheralmove();


                currentAngle = angles.firstAngle;
                telemetry.addData("currentAngle loop 2", "%.1f", currentAngle);
            }

            telemetry.addLine("null angle");
        }
    }


    public void move(){
        ControlConfig.update(gamepad1, gamepad2);
        double theta = Math.toRadians(currentAngle);

        telemetry.addData("CurrentAngle", currentAngle);
        telemetry.addData("Theta", theta);

        //update to change starting orientation if needed
        forward = gamepad1.left_stick_y; //left joystick down
        right = -gamepad1.left_stick_x; //left joystick left, adjusting for strafe
        clockwise = gamepad1.right_stick_x; //right joystick right (up on FTC Dashboard)

        temp = (forward * Math.cos(theta) - right * Math.sin(theta));
        side = (forward * Math.sin(theta) + right * Math.cos(theta));

        forward = temp;
        right = side;

        telemetry.addData("right: ", right);
        telemetry.addData("forward: ", forward);
        telemetry.addData("temp: ", temp);
        telemetry.addData("side: ", side);
        telemetry.addData("clockwise: ", clockwise);

        frontLeftV = forward + right + clockwise;
        rearLeftV = forward - right + clockwise;
        rearRightV = forward + right - clockwise;
        frontRightV = forward - right - clockwise;

        telemetry.addData("front left: ", frontLeft);
        telemetry.addData("rear left: ", rearLeft);
        telemetry.addData("rear right: ", rearRight);
        telemetry.addData("front right: ", frontRight);

        //add speed control here

        if (gamepad1.left_bumper) {
            powerMultiplier = .8;//FAST MODE
        } else if (gamepad1.right_bumper) {
            powerMultiplier = .3; //SLOW MODE
        } else {
            powerMultiplier = .6; //NORMAL MODE
        }

        frontLeft.setPower(frontLeftV * powerMultiplier);
        frontRight.setPower(frontRightV * powerMultiplier);
        rearLeft.setPower(rearLeftV * powerMultiplier);
        rearRight.setPower(rearRightV * powerMultiplier);



    }

    public void peripheralmove() {

//TODO add automations/safeties from benchmark
        ////////////////// auto intake ///////////////////////
        /*
        if (gamepad2.right_bumper && sampleColor != "red") {

            intakeRunning = true;
            intake.setPower(powerIn);

            if ((sampleColor == "blue") //check color blue
                    && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                    && (intakeRunning == true))//claw is open
            {
                intake.setPower(0);//Taking in sample
                gamepad1.rumble(500);
                intakeRunning = false;
                intakeArm.setPosition(IntakeArmHold);

            } else if ((sampleColor == "yellow")//check color yellow
                    && (sensorDistance.getDistance(DistanceUnit.CM) <= 2) //distance less than 2 cm
                    && (intakeRunning == true))//intake is running
            {
                intake.setPower(0);//intake is running
                gamepad1.rumble(100);
                intakeRunning = false;
                intakeArm.setPosition(IntakeArmHold);
            }
        } else {
            intakeRunning = false;
            intake.setPower(0);
        }

        if ((sampleColor == "red")
                && (sensorDistance.getDistance(DistanceUnit.CM) <= 5)) //distance less than 2 cm
        {
            intake.setPower(powerOut);//intake is running counterclockwise
            intakeRunning = true;
        }
        if (gamepad2.left_bumper) {
            intake.setPower(powerOut);//intake is running counterclockwise
            intakeRunning = true;
        }
*/
//////////////////////////MANUAL INTAKE CONTROLS///////////////////////////////////////////
        if (gamepad2.right_bumper) {
            intakeRunning = true;
            intake.setPower(powerIn);
        }
        else if (gamepad2.left_bumper) {
            intake.setPower(powerOut);//intake is running counterclockwise
            intakeRunning = true;
        }
        else {
            intakeRunning = false;
            intake.setPower(0);
        }

///////////////////////////////////SAMPLE COLOR DETECTION///////////////////////////

        if ((sensorColor.blue() > sensorColor.green()) && (sensorColor.blue() > sensorColor.red())) {
            sampleColor = "blue";
        }
        if ((sensorColor.red() > sensorColor.blue()) && (sensorColor.red() > sensorColor.green())) {
            sampleColor = "red";
        } else {
            sampleColor = "yellow";
        }



//////////////////////////////////////////linear slide///////////////////////


        if (gamepad2.y && intakeUp == true) {
            slideDown = false;
            linearSlideTarget = highBucketHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        if (gamepad2.x && intakeUp == true) {
            slideDown = false;
            linearSlideTarget = lowBucketHeight;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        // bring slide to ground
        if (gamepad2.left_stick_y > 0.5 && gamepad2.right_stick_y > 0.5 && clawIsOpen == true) {
            slideGoingDown = true;
            linearSlideTarget = 0;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        //slide has reached ground position
        if (touch.isPressed() == true && slideGoingDown == true) {
            slideDown=true;
            slideGoingDown = false;
            linearSlide.setTargetPosition(linearSlide.getCurrentPosition());
            linearSlide.setPower(0);
            ascentArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ascentArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlide.setTargetPosition(0);
            ascentArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        if (gamepad2.b && intakeUp == true) {
            slideDown = false;
            linearSlideTarget = highChamberHeight;//high chamber height
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }
        if (gamepad2.a && intakeUp == true) {
            slideDown = false;
            linearSlideTarget = highChamberReleaseHeight;//high chamber height
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(1);
        }



////////////////////////////////////////intake arm///////////////////////

        // brings arm to down/collect position
        if (gamepad2.dpad_down && slideDown == true) {
            servoPosition = IntakeArmDown;
            intakeUp = false;

            // brings arm to hold position
        } else if (gamepad2.dpad_left && slideDown == true) {
            servoPosition = IntakeArmHold;
            intakeUp = false;

            // brings arm to score/up position
        } else if (gamepad2.dpad_up) {
            servoPosition = IntakeArmUp;
            intakeUp = true;
        }

        intakeArm.setPosition(servoPosition);



////////////////////////////////////////manual claw controls///////////////////////

        if (gamepad2.left_trigger > 0.5) {
            claw.setPosition(ClawOpen);
            clawIsOpen = true;
        }

        if (gamepad2.right_trigger > 0.5) {
            claw.setPosition(ClawClosed);
            clawIsOpen = false;
        }


        ////////////////////////////////////////CLAW AUTOGRAB///////////////////////
        //make sure to raise linear slide above wall after grabbing
        if (sensorDistanceClaw.getDistance(DistanceUnit.CM) <= 5 && clawIsOpen==true && slideDown == true && clawLastClosed.seconds() > 1) {
            gamepad2.rumble(500);
            claw.setPosition(ClawClosed);//Claw Closed
            clawLastClosed.reset();
            clawIsOpen=false;
        }
        telemetry.addData("claw open", clawIsOpen);
        telemetry.addData("Distance(claw)", sensorDistanceClaw.getDistance(DistanceUnit.CM));


    }
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

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

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