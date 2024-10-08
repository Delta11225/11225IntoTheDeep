package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
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


@TeleOp
public class TeleOpTest extends OpMode {


    //HardwareITD robot;

    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;



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
    double powerMultiplier = 1;
    double deadZone = Math.abs(0.2);

    double temp;
    double side;

    double IMUAngle;
    double currentAngle;

    boolean clampIsClosed = false;
    boolean slideDown = true;
    boolean slowMode = false;

    boolean armIsScoring = false;

    @Override
    public void init() {
        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //robot = new HardwareITD(hardwareMap);
        rearLeft = hardwareMap.dcMotor.get("rear_left");
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight = hardwareMap.dcMotor.get("front_right");
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        rearRight = hardwareMap.dcMotor.get("rear_right");
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //initialize drive motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);



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




            if (orientation.getYaw(AngleUnit.DEGREES) < 0.0) {

                telemetry.update();
                move();

                currentAngle = orientation.getYaw(AngleUnit.DEGREES) + 360;
                telemetry.addData("currentAngle loop 1", "%.1f", currentAngle);
            }

            if (orientation.getYaw(AngleUnit.DEGREES) >= 0.0) {

                telemetry.update();
                move();

                currentAngle = orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("currentAngle loop 2", "%.1f", currentAngle);
            }

            telemetry.addLine("null angle");


    }




    public void move(){

        double theta = Math.toRadians(currentAngle);

        telemetry.addData("CurrentAngle", currentAngle);
        telemetry.addData("Theta", theta);

        //update to change starting orientation if needed
        forward = -gamepad1.left_stick_y;
        right = gamepad1.left_stick_x;
        clockwise = gamepad1.right_stick_x;

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

        telemetry.addData("front left: ", frontLeftV);
        telemetry.addData("rear left: ", rearLeftV);
        telemetry.addData("rear right: ", rearRightV);
        telemetry.addData("front right: ", frontRightV);

        // Handle speed control






        frontLeft.setPower(frontLeftV * powerMultiplier);
        frontRight.setPower(frontRightV * powerMultiplier);
        rearLeft.setPower(rearLeftV * powerMultiplier);
        rearRight.setPower(rearRightV * powerMultiplier);


    }


}