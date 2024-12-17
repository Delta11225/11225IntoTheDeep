package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.HardwareITD;

@Autonomous(preselectTeleOp = "TeleOpLM2")

public class AutoNetSide_3S extends LinearOpMode {

    RevBlinkinLedDriver lights;
    HardwareITD robot;

    double IntakeArmUp = .86;
    double IntakeArmHold = .6;
    double IntakeArmDown = .5;

    double ArmClawOpen = 0;
    double ArmClawClosed = 0.5;

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
        robot.ArmClaw.setPosition(ArmClawClosed);
        robot.SpecimenClaw.setPosition(ClawClosed);
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
        Pose2d startPose = new Pose2d(34, 66.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        //Trajectory Sequence for deploying preloaded sample
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
            .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.SpecimenClaw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
            .lineToLinearHeading(new Pose2d(57, 54, Math.toRadians(45)))
                .addSpatialMarker(new Vector2d(35, 66),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.linearSlide.setTargetPosition(highBucketHeight);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
            .waitSeconds(0.3)
            .forward(6)
            .waitSeconds(0.1)
                //drops preloaded sample into high basket
                .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
            .forward(0.0001)
            .waitSeconds(0.3)
            .back(6)

            .build();

        //picks up first yellow sample with claw arm
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
            .lineToLinearHeading(new Pose2d(53, 47, Math.toRadians(270)),
                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
                //lower arm
                .addDisplacementMarker(()->{
                    robot.SpecimenClaw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmDown);
                })
            .forward(0.00001)
            .waitSeconds(0.5)
                //close claw on 1st yellow sample
                .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.SpecimenClaw.setPosition(ClawClosed);
                })
            .forward(0.00001)
            .waitSeconds(0.5)
                //arm up
                .addDisplacementMarker(()->{
                    robot.SpecimenClaw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })

            .build();


        //goes to high bucket with 1st yellow sample
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())

            .lineToLinearHeading(new Pose2d(57, 54, Math.toRadians(45)),
                    SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
                //raises linear slide
                .addSpatialMarker(new Vector2d(53.5, 51),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.linearSlide.setTargetPosition(highBucketHeight);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
            .waitSeconds(0.3)
            .forward(6)
            .waitSeconds(0.1)
                //drops 1st yellow sample into high basket
                .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
            .forward(0.0001)
            .waitSeconds(0.3)
            .back(6)

            .build();

        //grabs second sample
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                //go to second sample
                .lineToLinearHeading(new Pose2d(63, 47, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                    //lower arm
                    .addDisplacementMarker(() ->{
                    robot.SpecimenClaw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmDown);
                })
                .forward(0.00001)
                .waitSeconds(0.5)
                //close claw on 2nd yellow sample
                    .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.SpecimenClaw.setPosition(ClawClosed);
                })
                .forward(0.00001)
                .waitSeconds(0.5)
                //arm up
                    .addDisplacementMarker(()->{
                    robot.SpecimenClaw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })

                .build();

        //put second sample in high basket
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(62, 50, Math.toRadians(45)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                    //raises linear slide
                    .addSpatialMarker(new Vector2d(53.5, 51),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.linearSlide.setTargetPosition(highBucketHeight);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .waitSeconds(0.3)
                .forward(6)
                .waitSeconds(0.1)
                    //drops 2nd yellow sample into high basket
                    .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                .forward(0.0001)
                .waitSeconds(0.3)
                .back(6)

            .build();

        //orients the robot properly (towards drivers) for TeleOp driver oriented controls
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(48,48, Math.toRadians(90)))

            .build();


        waitForStart();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);

      drive.followTrajectorySequence(traj1);
      returnSlideToGround();
      drive.followTrajectorySequence(traj2);
      drive.followTrajectorySequence(traj3);
      returnSlideToGround();
      drive.followTrajectorySequence(traj4);
      drive.followTrajectorySequence(traj5);
      returnSlideToGround();


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
