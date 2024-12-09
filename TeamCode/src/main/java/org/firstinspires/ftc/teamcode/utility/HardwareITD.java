package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class HardwareITD {

    public IMU imu;
    //declaring drive motors
    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    //declaring peripheral motors/sensors
    public DcMotor ascentArm;
    public DcMotor linearSlide;
    public Servo intakeArm;
    public CRServo intake;
    public Servo claw;
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public TouchSensor touch;
    RevBlinkinLedDriver lights;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    private final HardwareMap hwMap = null;
    private final ElapsedTime runtime = new ElapsedTime();

    // RoadRunner driver
    //public SampleMecanumDrive drive;
    //public TrajectoryGenerator generator;
    public HardwareITD(@NonNull HardwareMap hardwareMap) {

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        //To Do:  EDIT these two lines to match YOUR mounting configuration.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Define and initialize motors
        //initializing peripheral motors
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakeArm = hardwareMap.get(Servo.class, "intake_arm");
        ascentArm = hardwareMap.get(DcMotor.class, "ascent_arm");
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorV3"); //claw distance?
        sensorColor = hardwareMap.get(ColorSensor.class, "colorV3");
        touch = hardwareMap.get(TouchSensor.class, "touch");

        //LED sequence
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);

       //initializing drive motors
        rearLeft = hardwareMap.dcMotor.get("leftRear");
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft = hardwareMap.dcMotor.get("leftFront");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight = hardwareMap.dcMotor.get("rightFront");
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        rearRight = hardwareMap.dcMotor.get("rightRear");
        rearRight.setDirection(DcMotor.Direction.FORWARD);

    }

    }








