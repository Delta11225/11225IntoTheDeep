package org.firstinspires.ftc.teamcode.lessons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//@Disabled
@Autonomous
public class RRFirstTrajectory extends LinearOpMode {
    @Override

    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 28, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0,28, Math.toRadians(0)), true)

                .splineToSplineHeading(new Pose2d(-30,28.4, Math.toRadians(9)), Math.toRadians(9))
                .splineToSplineHeading(new Pose2d(-31.2,10.8, Math.toRadians(13)), Math.toRadians(13))
                .splineToSplineHeading(new Pose2d(-37.2,11.6, Math.toRadians(1)), Math.toRadians(1))
                .splineToSplineHeading(new Pose2d(-40.8,70.4, Math.toRadians(0)), Math.toRadians(0))
                .build();




        waitForStart();
        drive.followTrajectory(traj1);


    }
}
