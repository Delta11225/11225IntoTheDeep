package org.firstinspires.ftc.teamcode.lessons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
// @Disabled
@Autonomous
@Disabled
public class RRMockAutoChallenge extends LinearOpMode {


    @Override
    public void runOpMode(){


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(9.5, 66, Math.toRadians(270)); //STARTING POSE:
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(9.5, 48)) //hangs Specimen
                .lineToConstantHeading(new Vector2d(33, 48))//Goes towards first sample
                .lineToLinearHeading(new Pose2d(43.5, 33, Math.toRadians(315))) //Collects first sample
                .lineToLinearHeading(new Pose2d(50.5, 48, Math.toRadians(45))) //To basket
                .lineToLinearHeading(new Pose2d(60, 35, Math.toRadians(270))) //Collects second sample
                .lineToLinearHeading(new Pose2d(50.5, 48, Math.toRadians(45))) //To basket
                .lineToLinearHeading(new Pose2d(62, 24, Math.toRadians(0))) //Collects 3rd sample
                .lineToLinearHeading(new Pose2d(50.5, 48, Math.toRadians(45))) //To basket
                .setReversed(true).splineToLinearHeading(new Pose2d(25, 5, Math.toRadians(180)),Math.toRadians(180)) //PARK
                .build();



        waitForStart();

        drive.followTrajectorySequence(traj1);


    }
}

