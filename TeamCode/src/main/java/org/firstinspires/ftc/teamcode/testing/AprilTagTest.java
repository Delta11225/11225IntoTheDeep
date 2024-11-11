package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@Disabled
@TeleOp
public class AprilTagTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            if(result != null){
                if(result.isValid()){
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx",result.getTx());
                    telemetry.addData("ty",result.getTy());
                    telemetry.addData("Botpose",botpose.toString());

                }
            }
        }
    }
}
