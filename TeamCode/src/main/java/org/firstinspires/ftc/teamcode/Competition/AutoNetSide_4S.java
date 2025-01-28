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

@Autonomous(preselectTeleOp = "TeleopFINAL")

public class AutoNetSide_4S extends LinearOpMode {

    RevBlinkinLedDriver lights;
    HardwareITD robot;

    double IntakeArmUp = .86;
    double IntakeArmHold = .6;
    double IntakeArmDown = .5;

    double ArmClawOpen = 0;
    double ArmClawClosed = 0.5;

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
        Pose2d startPose = new Pose2d(35.375, 66.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        //Trajectory Sequence for deploying preloaded sample
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
            .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.SpecimenClaw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
            .lineToLinearHeading(new Pose2d(56, 55, Math.toRadians(45)))
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

        //picks up second yellow sample with claw arm
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(),Math.toRadians(270)) //(57, 54)
            .splineToLinearHeading(new Pose2d(51.5,47, Math.toRadians(270)), Math.toRadians(180),
                 SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                 SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
                .addSpatialMarker(new Vector2d(57,54),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawOpen);
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

            .build();

        //goes to high bucket with 2nd yellow sample
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56,55, Math.toRadians(45)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //raises linear slide
                .addSpatialMarker(new Vector2d(52,48),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.linearSlide.setTargetPosition(highBucketHeight);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .build();

        //approaches basket
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())


            .forward(4.5)
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

        //grabs third sample
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(60,46, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addSpatialMarker(new Vector2d(56,53),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawOpen);
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

                .build();

        //goes to high bucket with 3rd sample
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(51, 54, Math.toRadians(45)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //raises linear slide
                .addSpatialMarker(new Vector2d(61,48),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.linearSlide.setTargetPosition(highBucketHeight);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

               .build();


        //approaches basket
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end())

                .forward(0.00001)
                .waitSeconds(1.3)
                .forward(7)
                .waitSeconds(0.3)
                //drops 1st yellow sample into high basket
                .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                .forward(0.0001)
                .waitSeconds(0.3)
                .back(6)

            .build();

        //grabs 4th sample
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .lineToLinearHeading(new Pose2d(58.5, 41, Math.toRadians(300)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addSpatialMarker(new Vector2d(59,52),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawOpen);
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end(),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(51, 54, Math.toRadians(45)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //raises linear slide
                .addSpatialMarker(new Vector2d(57,42),() ->{
                    robot.intakeArm.setPosition(IntakeArmUp);
                    robot.ArmClaw.setPosition(ArmClawClosed);
                    robot.linearSlide.setTargetPosition(highBucketHeight);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

                .build();

        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())

                .forward(0.00001)
                .waitSeconds(1.3)
                .forward(7)
                .waitSeconds(0.3)
                //drops 1st yellow sample into high basket
                .addDisplacementMarker(()->{
                    robot.ArmClaw.setPosition(ArmClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                .forward(0.0001)
                .waitSeconds(0.3)
                .back(6)

                .build();






        waitForStart();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);

      drive.followTrajectorySequence(traj1);

      drive.followTrajectory(traj2);
      sleep(300);
      grabsample();
      sleep(300);
      drive.followTrajectory(traj3);
      sleep(300);
      drive.followTrajectorySequence(traj4);

      drive.followTrajectory(traj5);
      sleep(300);
      grabsample();
      sleep(300);
      drive.followTrajectory(traj6);
      //sleep(300);
      drive.followTrajectorySequence(traj7);

      drive.followTrajectorySequence(traj8);
      grabsample();
      drive.followTrajectory(traj9);
      drive.followTrajectorySequence(traj10);
    }

    public void grabsample(){

            robot.ArmClaw.setPosition(ArmClawOpen);

            //lower arm
            robot.intakeArm.setPosition(IntakeArmHold);
            sleep(600);
            robot.intakeArm.setPosition(IntakeArmDown);
            sleep(600);

            //close claw on 1st yellow sample
            robot.ArmClaw.setPosition(ArmClawClosed);
            sleep(500);

                //arm up
            robot.intakeArm.setPosition(IntakeArmUp);
            sleep(500);
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
