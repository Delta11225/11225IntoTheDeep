package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utility.HardwareITD;


@TeleOp

public class TeleOpTestITD extends LinearOpMode {

    HardwareITD robot;

    private ElapsedTime runtime = new ElapsedTime();

    double frontLeftV;
    double rearLeftV;
    double frontRightV;
    double rearRightV;
    double denominator;

    double forward;
    double right;
    double clockwise;

    double powerMultiplier = 1;


    double up;
    double side;

    double currentAngle;


    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new HardwareITD(hardwareMap);

        //initialize drive motors
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);

        waitForStart();

        currentAngle = 0;
        runtime.reset();
        robot.imu.resetYaw();


        // Loop and update the dashboard
        while (!isStopRequested()) {
            YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();

            telemetry.addData("currentAngle", "%.1f", currentAngle);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();

            currentAngle = orientation.getYaw(AngleUnit.DEGREES);

            move();

        }
    }

    public void move(){

        double theta = Math.toRadians(currentAngle);

        telemetry.addData("CurrentAngle", currentAngle);
        telemetry.addData("Theta", theta);

        //Orientation set for robot facing driver
        forward = gamepad1.left_stick_y; //left joystick down
        right = -gamepad1.left_stick_x*1.1; //left joystick left, adjusting for strafe
        clockwise = gamepad1.right_stick_x; //right joystick right (up on FTC Dashboard)

        up = (forward * Math.cos(theta) - right * Math.sin(theta)); //calculation of y'
        side = (forward * Math.sin(theta) + right * Math.cos(theta)); //calculation of x'

        forward = up;
        right = side;

        denominator = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(clockwise),1);
        frontLeftV = (forward + right + clockwise)/denominator;
        rearLeftV = (forward - right + clockwise)/denominator;
        rearRightV = (forward + right - clockwise)/denominator;
        frontRightV = (forward - right - clockwise)/denominator;

        telemetry.addData("front left: ", robot.frontLeft);
        telemetry.addData("rear left: ", robot.rearLeft);
        telemetry.addData("rear right: ", robot.rearRight);
        telemetry.addData("front right: ", robot.frontRight);

        // Handle speed control
        robot.frontLeft.setPower(frontLeftV * powerMultiplier);
        robot.frontRight.setPower(frontRightV * powerMultiplier);
        robot.rearLeft.setPower(rearLeftV * powerMultiplier);
        robot.rearRight.setPower(rearRightV * powerMultiplier);

        //add speed control here

        //fast mode
        if (gamepad1.left_bumper) {
            powerMultiplier = .7;}

        //slow mode
        else if (gamepad1.right_bumper){
            powerMultiplier= .15;
        }
        //normal mode
        else {
            powerMultiplier= .4;
        }

    }
}
