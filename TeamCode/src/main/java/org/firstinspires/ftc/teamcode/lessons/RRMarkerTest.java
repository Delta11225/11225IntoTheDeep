package org.firstinspires.ftc.teamcode.lessons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class RRMarkerTest extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(0, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-65, 20, Math.toRadians(180)))

                .addSpatialMarker(new Vector2d(-35, 20), () -> {
                    leftClaw.setPosition(leftClawOpened);
                    rightClaw.setPosition(rightClawOpened);
                })
                .back(0.0001)
                .addSpatialMarker(new Vector2d(-65,20), () -> {
                    leftClaw.setPosition(leftClawClosed);
                    rightClaw.setPosition(rightClawClosed);
                    })
                .waitSeconds(0.5)
                .back(5)
                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(90)))


                .build();//center spike mark

        //Init Servos
        leftClaw.setPosition(leftClawClosed);
        rightClaw.setPosition(rightClawClosed);

        waitForStart();

        drive.followTrajectorySequence(traj1);


    }

}
