package org.firstinspires.ftc.teamcode.lessons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoBlueFront extends LinearOpMode {
    public Servo leftClaw = null; // leftClaw = port 1
    public Servo rightClaw = null; // rightClaw = port 0
    double leftClawClosed = 0.8;
    double leftClawOpened = 0.4;
    double rightClawClosed = 0.1;
    double rightClawOpened = 0.4;

    @Override
    public void runOpMode(){
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-3.5, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
            // linear slide stuff here
            .lineToLinearHeading(new Pose2d(3.5, 33, Math.toRadians(270)))
            .waitSeconds(1.5)
            .lineToLinearHeading(new Pose2d(-49, 50, Math.toRadians(90)))
            .waitSeconds(1.5)
            .lineToLinearHeading(new Pose2d(67, 55, Math.toRadians(90)))//PARK yo net zone (no points)
            .build();


        waitForStart();
        drive.followTrajectorySequence(traj1);


    }

}