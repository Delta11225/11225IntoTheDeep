package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.HardwareITD;

@Autonomous(preselectTeleOp = "TeleopLM2")
public class AutoObservationSide_3S extends LinearOpMode {

    RevBlinkinLedDriver lights;
    HardwareITD robot;

    double IntakeArmUp = .86;
    double IntakeArmHold = .6;
    double IntakeArmDown = .5;

    double powerIn = 1.0;
    double powerOut = -1.0;

    double ClawOpen = 0.4;
    double ClawClosed = 0.85;

    int highBucketHeight = 3600;
    int lowBucketHeight = 1675;
    int highChamberHeight = 2350;
    int highChamberReleaseHeight = 1200;

    @Override
    public void runOpMode(){

        robot = new HardwareITD(hardwareMap);

        //initialize necessary motors & servos to start positions
        robot.claw.setPosition(ClawClosed);
        robot.intakeArm.setPosition(IntakeArmUp);

        robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlide.setDirection(DcMotor.Direction.REVERSE);
        robot.linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.linearSlide.setPower(0);

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Set starting Pose for robot (coordinate and heading)
        Pose2d startPose = new Pose2d(-13.125, 66.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        //Trajectory Sequence for deploying preloaded specimen
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                //closing claw and lifting arm up
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                //approach submersible
                .lineToConstantHeading(new Vector2d(1.5, 29))
                //start lifting slide when robot is at (-12,64) which is close to start pose
                .addSpatialMarker(new Vector2d(-12, 64), () -> {
                    robot.linearSlide.setTargetPosition(2250);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                .strafeLeft(5) //position is (1.5, 24)
                .build();

        //pushes first blue sample into observation zone, also grabs second specimen
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(),Math.toRadians(100))

                .splineToSplineHeading(
                        new Pose2d(-16,46, Math.toRadians(90)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(46, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(
                        new Pose2d(-36,28, Math.toRadians(180)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(46, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-36,16), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-43,8), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(51, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-50,16), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(51, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-50,56), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(38, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-38.5,40), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-29.5,55), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //GRAB!!!
                .splineToConstantHeading(
                        new Vector2d(-29.5,68), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
//fixed version V2

                .build();

        //goes to submersible holding second specimen
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), Math.toRadians(270))
                .splineToConstantHeading(
                        new Vector2d(-28,64), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(51, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(
                        new Pose2d(-15,43, Math.toRadians(0)), Math.toRadians(360),
                        SampleMecanumDrive.getVelocityConstraint(46, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //lifts linear slide while moving
                .addSpatialMarker(new Vector2d(-42, 59), () -> {
                    robot.linearSlide.setTargetPosition(1750);//was 2050
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToConstantHeading(
                        new Vector2d(-3,31), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //clips second specimen
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(875);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .build();

        //goes to grab 3rd specimen //GRAB!!!
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-28, 60.5, Math.toRadians(180)))
                .strafeRight(6.8)
                .build();

        //hangs 3rd specimen
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(3, 36, Math.toRadians(359)))
                //lifts linear slide while moving
                .addSpatialMarker(new Vector2d(-42, 59), () -> {
                    robot.linearSlide.setTargetPosition(1750);//was 2050
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                //moves closer to submersible
                .strafeRight(5)
                //clips second specimen
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(875);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .build();

        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                //PARK IN OBSERVATION ZONE FACING AWAY FROM DRIVERS
                .strafeLeft(5)
                .lineToLinearHeading(new Pose2d(-60, 59, Math.toRadians(270)))
                .build();




        waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
        drive.followTrajectorySequence(traj1);
        deploySpecimen();
        drive.followTrajectory(traj2);
        retrieveObservationSpecimen();
        drive.followTrajectory(traj3);
        deploySpecimen();
        drive.followTrajectorySequence(traj4);
        retrieveObservationSpecimen();
        drive.followTrajectorySequence(traj5);
        deploySpecimen();
        drive.followTrajectorySequence(traj6);


    }
    public void deploySpecimen() {
        //open claw
        robot.claw.setPosition(ClawOpen);
        //return slide to ground
        robot.linearSlide.setTargetPosition(0);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linearSlide.setPower(1);
        while (robot.linearSlide.isBusy() || robot.touch.isPressed()==false) {
            if(robot.touch.isPressed()){
                robot.linearSlide.setTargetPosition(robot.linearSlide.getCurrentPosition());
            }
            telemetry.addData("slide Power",robot.linearSlide.getPower());
            telemetry.addData("slide encoder",robot.linearSlide.getCurrentPosition());
            telemetry.addData("slide target",robot.linearSlide.getTargetPosition());
            telemetry.addData("touch state",robot.touch.isPressed());
            telemetry.update();
        }
    }

    public void retrieveObservationSpecimen(){
        //grab specimen hanging on wall
        robot.claw.setPosition(ClawClosed);
        robot.intakeArm.setPosition(IntakeArmUp);
        sleep(500);
        //lift linear slide up to clear wall with specimen
        robot.linearSlide.setTargetPosition(700);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linearSlide.setPower(1);
        while (robot.linearSlide.isBusy()) {
            telemetry.addData("slide Power", robot.linearSlide.getPower());
            telemetry.addData("slide encoder", robot.linearSlide.getCurrentPosition());
            telemetry.addData("slide target", robot.linearSlide.getTargetPosition());
            telemetry.update();
        }
        sleep(100);
    }
}
