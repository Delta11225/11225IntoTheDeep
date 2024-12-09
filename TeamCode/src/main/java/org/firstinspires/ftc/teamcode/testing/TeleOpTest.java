package org.firstinspires.ftc.teamcode.testing;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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



import java.util.Locale;

@Disabled
@TeleOp
public class TeleOpTest extends OpMode {


    //HardwareITD robot;

    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public Servo claw;


    // The IMU sensor object
    IMU imu;


    private ElapsedTime runtime = new ElapsedTime();

    double frontLeftV;
    double rearLeftV;
    double frontRightV;
    double rearRightV;

    double forward;
    double right;
    double clockwise;
    int linearSlideZeroOffset = 0;
    double powerMultiplier = 0.7;
    double deadZone = Math.abs(0.2);


    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    public CRServo intake = null;
    boolean intakeRunning = true;
    String sampleColor = "none";
    boolean intakeUp = true;
    private DcMotor linearSlide;
    private int linearSlideTarget = 0;
    private int linearSlideZero = 0;
    boolean sliderunning = false;
    TouchSensor touch;
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

    boolean clampIsClosed = false;
    boolean slideDown = true;
    boolean slowMode = false;
    boolean slideGoingDown = false;
    boolean armIsScoring = false;

    //from AscentArmAutoTest
    private DcMotor ascentArm;
    int armHang = 234;
    int armHook = 8515;
    int store = 0;

    @Override
    public void init() {
        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        //To Do:  EDIT these two lines to match YOUR mounting configuration.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

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
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3");

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
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3");


        ///////////////Ascent Arm//////////////////////
        ascentArm = hardwareMap.get(DcMotor.class, "ascent_arm");
        ascentArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascentArm.setZeroPowerBehavior(BRAKE);
        //Turn on Run to Position and set initial target at store = 0
        ascentArm.setTargetPosition(store);
        ascentArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void start() {
        currentAngle = 0;
        runtime.reset();
        imu.resetYaw();
    }

    @Override
    public void loop() {

        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);


        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.update();

        currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        move();
        peripheral();

    }

    /////////////////////////move sequence//////////////////////////
    public void move() {

        double theta = Math.toRadians(currentAngle);

        telemetry.addData("CurrentAngle", currentAngle);
        telemetry.addData("Theta", theta);

        //Orientation set for robot facing driver
        forward = gamepad1.left_stick_y; //left joystick down
        right = -gamepad1.left_stick_x * 1.1; //left joystick left, adjusting for strafe
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

        // Handle speed control
        frontLeft.setPower(frontLeftV * powerMultiplier);
        frontRight.setPower(frontRightV * powerMultiplier);
        rearLeft.setPower(rearLeftV * powerMultiplier);
        rearRight.setPower(rearRightV * powerMultiplier);

        //add speed control here

        if (gamepad1.left_bumper) {
            powerMultiplier = .8;//FAST MODE
        } else if (gamepad1.right_bumper) {
            powerMultiplier = .3; //SLOW MODE
        } else {
            powerMultiplier = .6; //NORMAL MODE
        }

/////////////////////////////////////Ascent Arm Auto//////////////////////////////////////////////////////
        if (gamepad1.dpad_down & gamepad1.a) {
            ascentArm.setTargetPosition(store);
            ascentArm.setPower(0.5);
        }

        if (gamepad1.dpad_left & gamepad1.b) {
            ascentArm.setPower(1);
            ascentArm.setTargetPosition(armHook);
        }

        if (gamepad1.dpad_up & gamepad1.y) {
            ascentArm.setPower(1);
            ascentArm.setTargetPosition(armHang);

        }

        telemetry.addData("encoder", ascentArm.getCurrentPosition());
        telemetry.update();
    }

    public void peripheral() {

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
               if (gamepad2.left_trigger > .5) {
                    intakeRunning = true;
                    intake.setPower(powerIn);
                }
                else if (gamepad2.right_trigger > .5) {
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

        telemetry.addData("Color vals, r", sensorColor.red());
        telemetry.addData("Color vals, g", sensorColor.green());
        telemetry.addData("Color vals, b", sensorColor.blue());
        telemetry.addData("Distance(cm)", sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Color Detected", sampleColor);
        telemetry.addData("intake power", intake.getPower());

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
        if (gamepad2.left_stick_y > 0.5 && gamepad2.right_stick_y > 0.5) {
            slideGoingDown = true;
            linearSlideTarget = 0;
            linearSlide.setTargetPosition(linearSlideTarget);
            linearSlide.setPower(0.8);
        }

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
        telemetry.addData("encoder", linearSlide.getCurrentPosition());


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

        telemetry.addData("position", servoPosition);
        telemetry.addData("intake arm", intakeArm.getPosition());

////////////////////////////////////////manual claw controls///////////////////////

        if (gamepad2.left_trigger > 0.5) {
            claw.setPosition(ClawOpen);
        }

        if (gamepad2.right_trigger > 0.5) {
            claw.setPosition(ClawClosed);
        }


        ////////////////////////////////////////CLAW AUTOGRAB///////////////////////
        //make sure to raise linear slide above wall after grabbing

    }
}



    /////end of peripheral move////////


