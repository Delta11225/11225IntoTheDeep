package org.firstinspires.ftc.teamcode.utility;

/**
 * This is a class that holds all necessary constants for the robot. Put "magic numbers" here.
 * This class cannot be instantiated. Do not inherit from it.
 * To add a new constant, simply declare a new public final static variable and assign the
 * appropriate value.
 */
public abstract class Constants {

    public final static double fastMultiplier = 1.0;
    public final static double normalMultiplier = 0.45;
    public final static double slowMultiplier = 0.25;
    public final static double superSlowMultiplier = 0.25;

    public final static double droneLaunch = 0.0;

    public final static double droneHold = 0.52;

    public final static double armCollectPosition = 0.83;
    public final static double armScoringPosition = 0.55;
    public final static double armHoldPosition = 0.88;
    public final static double armTrussHeight = 1;

    public final static double clampClosedPosition = 0.3;
    public final static double clampOpenPosition = 0.05;
//for auto
    public final static int linearSlideAutonomousDeploy = 2600;
    public final static int linearSlideAutonomousDrop = 1800;
    //for teleop
    public final static int linearSlideAutomatedDeployLow = 3000;//2771 old value

    public final static int linearSlideLowSafety = 2500;
    public final static int linearSlideAutomatedDeployHigh = 4109;


    public final static int scissorHookHeightLeft = 5650;
    public final static int scissorHookHeightRight = 5150;

    public final static int scissorLiftHeightLeft = scissorHookHeightLeft-3000;
    public final static int scissorLiftHeightRight = scissorHookHeightRight-3000;

    //left 2724
    //right 2210

}