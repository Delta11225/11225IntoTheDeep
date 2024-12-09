package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp
public class IfElseTestPSL extends OpMode {
   //Declare OpMode members
   public DcMotor motor1 = null;

   //Code to run ONCE when the driver hits INIT
   @Override
   public void init(){
      //Define and initialize motor
      motor1 = hardwareMap. get(DcMotor.class, "motor_1");
   }
   //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   @Override
   public void loop(){
      if(gamepad1.left_stick_x>=0.5){
         telemetry.addLine("y stick x over 0.5");
         motor1.setPower(1);
      }
      else if(gamepad1.left_stick_x<=-0.5){
         telemetry.addLine("y stick y under -0.5");
         motor1.setPower(-0.5);
      }
      else{
         telemetry.addLine("none pushed or pulled");
         motor1.setPower(0);
      }
      //telemetry.addData("a button", gamepad1.a);
   }

}
