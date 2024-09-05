package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LogicalOperatorsPSL extends OpMode {
public DcMotor motor1 = null;
   //Code to run ONCE when the driver hits INIT
   @Override
   public void init(){
   motor1 = hardwareMap.get(DcMotor.class, "motor_1");
   }
   //Code to run REPEATED after the driver hits PLAY but before they hit STOP
   @Override
   public void loop(){
      if (gamepad1.a && gamepad1.b) {
         telemetry.addData("Status", "TRUE AB");
         motor1.setPower(1);
      } else if (gamepad1.x && gamepad1.y) {
         telemetry.addData("Status", "TRUE XY");
         motor1.setPower(-1);
      }
      else {
         telemetry.addData("Status", "FALSE");
         motor1.setPower(0);
      }
   }

}
