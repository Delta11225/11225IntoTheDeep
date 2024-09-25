package org.firstinspires.ftc.teamcode.lessons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous
public class RRFirstTrajectory extends LinearOpMode {
    @Override

    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .strafeRight(30)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(40)
                .build();

        waitForStart();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
    }
}
