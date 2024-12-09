package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.HardwareITD;

@Autonomous(preselectTeleOp = "TeleOpTestLM1")

public class AutoNetSide extends LinearOpMode {

    HardwareITD robot;

    double IntakeArmUp = .86;
    double IntakeArmHold = .6;
    double IntakeArmDown = .5;

    double powerIn = 1.0;
    double powerOut = -1.0;

    double ClawOpen = 0.4;
    double ClawClosed = 0.8;

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




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Set starting Pose for robot (coordinate and heading)
        Pose2d startPose = new Pose2d(18, 66.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        /////Trajectory Sequence for deploying preloaded specimen
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
            .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
            .lineToConstantHeading(new Vector2d(4, 29))
            .addSpatialMarker(new Vector2d(16, 62), () -> {
                robot.linearSlide.setTargetPosition(highChamberHeight);
                robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.linearSlide.setPower(1);
                })

            .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })

            .strafeLeft(10)//coordinate (4, 39)
            .build();
        //picks up first yellow sample with intake arm
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
            .lineToLinearHeading(new Pose2d(30, 35, Math.toRadians(340)))
                .addSpatialMarker(new Vector2d(6, 37),() ->{
                    robot.intakeArm.setPosition(IntakeArmDown);
                    robot.intake.setPower(powerIn);
                })

            .forward(4)
            .waitSeconds(0.4)
            .back(4)
            .waitSeconds(0.5)
                .addDisplacementMarker(()->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.intake.setPower(0);
                })
                // Approaches high basket
                .lineToLinearHeading(new Pose2d(52,59, Math.toRadians(45)))
                // raises slide to high bucket while approaching basket
                .addSpatialMarker(new Vector2d(31, 36), () -> {
                    robot.claw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.linearSlide.setTargetPosition(highBucketHeight);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .forward(4)
                .waitSeconds(0.5)
                .forward(0.001)
                .addDisplacementMarker(()->{
                    //drops off first sample in basket
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.intake.setPower(powerOut);
                })

                .forward(0.001)
                .waitSeconds(1)
                .back(3)
                .addDisplacementMarker(()->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.intake.setPower(0);
                })

            .build();

        //grabs second sample
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                //go to second sample
            .lineToLinearHeading(new Pose2d(35,27, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(50, 55),() ->{
                    robot.intakeArm.setPosition(IntakeArmHold);
                    robot.intake.setPower(powerIn);
                })
                //grab sample with intake
            .forward(2)
            .waitSeconds(0.1)
            .addDisplacementMarker(()->{
                robot.intakeArm.setPosition(IntakeArmDown);
                robot.intake.setPower(powerIn);
                })
            .forward(4)
            .waitSeconds(0.4)
            .back(4)
            .waitSeconds(0.6)
            .addDisplacementMarker(()->{
                robot.intakeArm.setPosition(IntakeArmUp);
                robot.intake.setPower(0);
                })

                .build();

        //put second sample in high basket
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
            .lineToLinearHeading(new Pose2d(51,58, Math.toRadians(45)))
            // raising slide to high bucket while approaching basket
            .addSpatialMarker(new Vector2d(39, 31), () -> {
                robot.claw.setPosition(ClawClosed);
                robot.intakeArm.setPosition(IntakeArmUp);
                robot.linearSlide.setTargetPosition(highBucketHeight);
                robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.linearSlide.setPower(1);
                })
            .forward(4)
            .waitSeconds(1)
            .forward(2)
            .addDisplacementMarker(()->{
                //drops second sample in basket
                robot.intakeArm.setPosition(IntakeArmUp);
                robot.intake.setPower(powerOut);
                })

            .forward(0.001)
            .waitSeconds(1)
            .back(3)
            .addDisplacementMarker(()->{
                robot.intakeArm.setPosition(IntakeArmUp);
                robot.intake.setPower(0);
                })

            .build();

        //orients the robot properly (towards drivers) for TeleOp driver oriented controls
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(48,48, Math.toRadians(90)))

            .build();


        waitForStart();

      drive.followTrajectorySequence(traj1);
      deploySpecimen();
      drive.followTrajectorySequence(traj2);
      returnSlideToGround();
      drive.followTrajectorySequence(traj3);
      drive.followTrajectorySequence(traj4);
      returnSlideToGround();
      drive.followTrajectorySequence(traj5);

    }

    //class made specifically
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
    public void returnSlideToGround() {


        //return slide to ground
        robot.linearSlide.setTargetPosition(0);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linearSlide.setPower(1);
        while (robot.linearSlide.isBusy() || robot.touch.isPressed() == false) {
            if (robot.touch.isPressed()) {
                robot.linearSlide.setTargetPosition(robot.linearSlide.getCurrentPosition());
            }
            telemetry.addData("slide Power", robot.linearSlide.getPower());
            telemetry.addData("slide encoder", robot.linearSlide.getCurrentPosition());
            telemetry.addData("slide target", robot.linearSlide.getTargetPosition());
            telemetry.addData("touch state", robot.touch.isPressed());
            telemetry.update();
        }
    }
}
