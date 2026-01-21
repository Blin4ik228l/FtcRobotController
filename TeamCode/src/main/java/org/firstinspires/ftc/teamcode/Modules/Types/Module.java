package org.firstinspires.ftc.teamcode.Modules.Types;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.Delays;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.kPIDS;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.ServoPositions;
import org.firstinspires.ftc.teamcode.Modules.Types.Interfaces.InterIter;
import org.firstinspires.ftc.teamcode.Modules.Types.Interfaces.InterResetTime;
import org.firstinspires.ftc.teamcode.Modules.Types.Interfaces.InterShow;
import org.firstinspires.ftc.teamcode.Modules.Types.Interfaces.InterUpdate;


public class Module implements InterShow, InterResetTime, InterIter,Delays, AnotherConsts, kPIDS, ServoPositions{
     public HardwareMap hardwareMap;
     public Telemetry telemetry;
     public VoltageSensorClass voltageSensor;
     public int iterationCount = 1;
     public ElapsedTime innerRunTime;

     public Module(OpMode op){
          this.hardwareMap = op.hardwareMap;
          this.telemetry = op.telemetry;
          this.voltageSensor = new VoltageSensorClass();
          this.innerRunTime = new ElapsedTime();
     }

     @Override
     public void showData() {

     }

     @Override
     public void resetTimer() {
          this.innerRunTime.reset();
     }

     @Override
     public void setIteration(int iterationCount) {
          this.iterationCount = iterationCount;
     }

     public class VoltageSensorClass implements InterShow, InterUpdate {
          private double curVoltage;
          private double MAX_VOL;
          private double kPower;

          public double getkPower() {
               return kPower;
          }

          public double getCurVoltage() {
               return curVoltage;
          }

          public double getMAX_VOL() {
               return MAX_VOL;
          }

          @Override
          public void update() {
               if (iterationCount % 10 == 0){
                    double result = Double.POSITIVE_INFINITY;

                    for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                         double voltage = sensor.getVoltage();
                         if (voltage > 0) {
                              result = Math.min(result, voltage);
                         }
                    }

                    curVoltage = result;
                    if(curVoltage > MAX_VOL) MAX_VOL = curVoltage;

                    kPower = MAX_VOL / curVoltage;
               }
          }

          @Override
          public void showData() {
               telemetry.addLine("===VoltageClass===");
               telemetry.addData("H size", hardwareMap.size());
               telemetry.addData("Vol", curVoltage);
               telemetry.addData("Max vol", MAX_VOL);
               telemetry.addData("KP", kPower);
               telemetry.addLine();
          }
     }
}
