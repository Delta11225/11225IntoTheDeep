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

@Autonomous(preselectTeleOp = "TeleopFINAL4S")
public class AutoObservationSide_4S_V4 extends LinearOpMode {

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


        //Deploys preloaded specimen
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                //closing claw and lifting arm up
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })

                //approaches submersible
                .lineToConstantHeading(new Vector2d(1.5, 29))

                //start lifting slide when robot is at (-12,64)
                .addSpatialMarker(new Vector2d(-12, 64), () -> {
                    robot.linearSlide.setTargetPosition(2250);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })

                .build();

        //pushes first blue sample into observation zone, also grabs second specimen
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(),Math.toRadians(100))

                //start lowing slide and opening claw while robot is splining
                .addSpatialMarker(new Vector2d(-1.5, 36), () -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);

                })
                .splineToSplineHeading(
                        new Pose2d(-16,46, Math.toRadians(90)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(
                        new Pose2d(-36,28, Math.toRadians(180)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-36,16), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(42, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-41.25,8), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-46.5,16), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-46.5,48), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-46.5,44), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-46.6,16), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-50,8), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-54,16), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-54,48), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-55,44), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-42,47), Math.toRadians(15),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-30,68.3), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //GRAB SECOND SPECIMEN!!
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                //(-30, 68.3)
                .build();

        //hangs second specimen
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), Math.toRadians(0))
                //raises slide while going to hang specimen
                .addSpatialMarker(new Vector2d(-29, 68), () -> {
                    robot.linearSlide.setTargetPosition(1800);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToConstantHeading(
                        new Vector2d(-23,68.3), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-15,63), Math.toRadians(290),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(
                        new Pose2d(-3,26, Math.toRadians(0)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(875);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

                .build();

        //coming back to grab 3rd specimen
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), Math.toRadians(270))
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                .splineToConstantHeading(
                        new Vector2d(-7,34), Math.toRadians(160),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addSpatialMarker(new Vector2d(-8, 35), () -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToSplineHeading(
                        new Pose2d(-28,62, Math.toRadians(180)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-28,65), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //GRAB THIRD SPECIMEN!!!
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })

                .build();

        //hangs 3rd specimen
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), Math.toRadians(90))

                //raises slide while going to hang specimen
                .addSpatialMarker(new Vector2d(-29, 68), () -> {
                    robot.linearSlide.setTargetPosition(1800);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToConstantHeading(
                        new Vector2d(-23,68.3), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-15,63), Math.toRadians(290),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(
                        new Pose2d(-2,26, Math.toRadians(0)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(875);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

                .build();

            //goes to grab 4th specimen
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), Math.toRadians(270))
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                .splineToConstantHeading(
                        new Vector2d(-7,34), Math.toRadians(160),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addSpatialMarker(new Vector2d(-8, 35), () -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToSplineHeading(
                        new Pose2d(-28,62, Math.toRadians(180)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-28,65), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                .build();

        //hangs 4rd specimen
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end(), Math.toRadians(90))
                //raises slide while going to hang specimen
                .addSpatialMarker(new Vector2d(-29, 68), () -> {
                    robot.linearSlide.setTargetPosition(1800);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToConstantHeading(
                        new Vector2d(-23,68.3), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-15,63), Math.toRadians(290),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(
                        new Pose2d(-7,26, Math.toRadians(0)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(875);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })


                .build();




        waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
        drive.followTrajectorySequence(traj1);

        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);

        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        deploySpecimen();


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
        sleep(400);
        //lift linear slide up to clear wall with specimen
        robot.linearSlide.setTargetPosition(200);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linearSlide.setPower(1);
        while (robot.linearSlide.isBusy()) {
            telemetry.addData("slide Power", robot.linearSlide.getPower());
            telemetry.addData("slide encoder", robot.linearSlide.getCurrentPosition());
            telemetry.addData("slide target", robot.linearSlide.getTargetPosition());
            telemetry.update();
        }

    }
    public void resetSlide() {
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

}


