package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.HardwareITD;

@Autonomous
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

            .strafeLeft(10)
            .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
            .lineToLinearHeading(new Pose2d(30, 35, Math.toRadians(340)))
                .addSpatialMarker(new Vector2d(25, 35),() ->{
                    robot.intakeArm.setPosition(IntakeArmDown);
                    robot.intake.setPower(powerIn);
                })
            .forward(4)
            .waitSeconds(0.4)
            .back(4)
            .build();



        waitForStart();

      drive.followTrajectorySequence(traj1);
      deploySpecimen();
      drive.followTrajectorySequence(traj2);


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
}
