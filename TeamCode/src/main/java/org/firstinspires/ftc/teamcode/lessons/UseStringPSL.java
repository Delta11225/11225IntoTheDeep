package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp()


public class UseStringPSL extends OpMode {
   @Override
   public void init(){
      String myName = "Pierson, Sophia, Lillian";
      int grade = 9;
      telemetry.addData("Hello", myName);
      telemetry.addData("Grade", grade);
   }
   @Override
   public void loop(){

   }
}





