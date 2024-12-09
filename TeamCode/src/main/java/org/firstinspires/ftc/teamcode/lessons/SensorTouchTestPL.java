package org.firstinspires.ftc.teamcode.lessons;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
@TeleOp
public class SensorTouchTestPL extends OpMode {
   TouchSensor touch;
   Servo servo1;
   DcMotor motor1;

   @Override
   public void init() {
      touch = hardwareMap.get(TouchSensor.class, "touch");
      servo1 = hardwareMap.get(Servo.class,"servo_1");
      motor1 = hardwareMap.get(DcMotor.class, "motor_1");
   }

   @Override
   public void loop() {
      telemetry.addData("Happiness Status", "Programming is fun!");
      if (touch.isPressed()) {
         telemetry.addData("Touch Sensor", "PRESSED");
         motor1.setPower(0.75);
      }
      else {
         telemetry.addData("Touch Sensor", "NOT PRESSED");
         motor1.setPower(0);
      }
   }
}
