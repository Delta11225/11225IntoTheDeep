package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTestPSL extends OpMode {
   public Servo servo1 = null;

   @Override
   public void init() {
      servo1 = hardwareMap.get(Servo.class, "servo_1");
   }

   @Override
   public void loop() {
      if (gamepad1.y) {
         telemetry.addData("Button Y", "Pressed");
         servo1.setPosition(0.5);
      }
      else if (gamepad1.b) {
         telemetry.addData("Button B", "Pressed");
         servo1.setPosition(1);
      }
      else servo1.setPosition(0);
   }
}
