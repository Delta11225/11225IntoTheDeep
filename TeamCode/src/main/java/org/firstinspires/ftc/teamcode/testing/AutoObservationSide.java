package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.HardwareITD;

@Autonomous(preselectTeleOp = "TeleOpLM1")
@Disabled
public class AutoObservationSide extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(-14.5, 66.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        /////Trajectory Sequence for deploying preloaded specimen
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                //closing claw and lifting arm up
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawClosed);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                //approach submersible
                .lineToConstantHeading(new Vector2d(0, 28))
                //start lifting slide when robot is at (-12,64) which is close to start pose
                .addSpatialMarker(new Vector2d(-12, 64), () -> {
                    robot.linearSlide.setTargetPosition(highChamberHeight);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .addDisplacementMarker(()->{
                    robot.claw.setPosition(ClawOpen);
                    robot.intakeArm.setPosition(IntakeArmUp);
                })
                .strafeLeft(5)
                .build();


                // Grabbing first sample from the observation zone
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                // robot approaches preloaded specimen in observation zone
                .lineToLinearHeading(new Pose2d(-41, 61, Math.toRadians(180)))
                .strafeRight(6,
                    SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                //goes to submersible holding second specimen
                .lineToLinearHeading(new Pose2d(-3.5, 36, Math.toRadians(0)))
                //lifts linear slide while moving
                .addSpatialMarker(new Vector2d(-42, 59), () -> {
                    robot.linearSlide.setTargetPosition(2050);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                //moves closer to submersible
                .strafeRight(5.75)
                //clips second specimen
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(900);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                //PARK IN OBSERVATION ZONE FACING DRIVERS
                .strafeLeft(5)
                .lineToLinearHeading(new Pose2d(-60, 59, Math.toRadians(90)))
                .build();







        waitForStart();

        drive.followTrajectorySequence(traj1);
        deploySpecimen();
        drive.followTrajectorySequence(traj2);
        retrieveObservationSpecimen();
        drive.followTrajectorySequence(traj3);
        deploySpecimen();
        drive.followTrajectorySequence(traj4);




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
