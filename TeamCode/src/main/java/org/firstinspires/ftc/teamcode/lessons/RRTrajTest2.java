package org.firstinspires.ftc.teamcode.lessons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@Autonomous
@Disabled
public class RRTrajTest2 extends LinearOpMode {
    @Override

    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 28, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0,28, Math.toRadians(0)),Math.toRadians(100))

                .splineToConstantHeading(new Vector2d(-16,37), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-36,28),  Math.toRadians(260))
                .splineToConstantHeading(new Vector2d(-36,16), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-43,8), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-50,16), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-50,62, Math.toRadians(0)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-43,50), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-38,60, Math.toRadians(180)),Math.toRadians(90))
                .build();




        waitForStart();
        drive.followTrajectory(traj1);


    }
}
