package org.firstinspires.ftc.teamcode.lessons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RRMockAutoChallenge extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(9.5, 60, Math.toRadians(270)); //STARTING POSE:
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(9.5, 33)) //hangs Specimen
                .lineToConstantHeading(new Vector2d(33, 33))//Goes towards first sample
                .addDisplacementMarker(10,() -> {
                    leftClaw.setPosition(leftClawOpened);
                    rightClaw.setPosition(rightClawOpened);
                })
                .lineToLinearHeading(new Pose2d(43.5, 33, Math.toRadians(315))) //Collects first sample
                .addDisplacementMarker(() -> {
                    leftClaw.setPosition(leftClawClosed);
                    rightClaw.setPosition(rightClawClosed);
                })
                .lineToLinearHeading(new Pose2d(50.5, 48, Math.toRadians(45))) //To basket
                .lineToLinearHeading(new Pose2d(60, 35, Math.toRadians(270))) //Collects second sample
                .lineToLinearHeading(new Pose2d(50.5, 48, Math.toRadians(45))) //To basket
                .lineToLinearHeading(new Pose2d(62, 24, Math.toRadians(0))) //Collects 3rd sample
                .lineToLinearHeading(new Pose2d(50.5, 48, Math.toRadians(45))) //To basket
                .setReversed(true).splineToLinearHeading(new Pose2d(25, 5, Math.toRadians(180)),Math.toRadians(180)) //PARK
                .build();




        leftClaw.setPosition(leftClawClosed);
        rightClaw.setPosition(rightClawClosed);

        waitForStart();

        drive.followTrajectorySequence(traj1);


    }
}

