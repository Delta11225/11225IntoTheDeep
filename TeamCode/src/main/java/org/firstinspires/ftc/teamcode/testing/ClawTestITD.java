package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.HardwareITD;

//@Disabled
@TeleOp
public class ClawTestITD extends OpMode {
   HardwareITD robot;


   @Override
   public void init() {
      robot = new HardwareITD(hardwareMap);
   }
   @Override
   public void loop() {
      if (gamepad1.y) {
         telemetry.addData("Button Y", "Pressed");
         robot.claw.setPosition(0.4);
      }
      if (gamepad1.b) {
         telemetry.addData("Button B", "Pressed");
         robot.claw.setPosition(0.825);
      }
   }
}


