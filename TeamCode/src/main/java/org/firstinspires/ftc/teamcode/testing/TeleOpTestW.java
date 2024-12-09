/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utility.HardwareITD;


@TeleOp

public class TeleOpTestW extends LinearOpMode {

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

        // Handle speed control
        robot.frontLeft.setPower(frontLeftV * powerMultiplier);
        robot.frontRight.setPower(frontRightV * powerMultiplier);
        robot.rearLeft.setPower(rearLeftV * powerMultiplier);
        robot.rearRight.setPower(rearRightV * powerMultiplier);

        //add speed control here

        //fast mode
        if (gamepad1.left_bumper) {
            powerMultiplier = .8;}

        //slow mode
        else if (gamepad1.right_bumper){
            powerMultiplier= .3;
        }
        //normal mode
        else {
            powerMultiplier= .6;
        }

    }
}
