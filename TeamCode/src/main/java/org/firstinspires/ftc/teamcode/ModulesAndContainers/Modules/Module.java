package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.Delays;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.kPIDS;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.ServoPositions;


public abstract class Module implements  Delays, AnotherConsts, kPIDS, ServoPositions{
     protected HardwareMap hardwareMap;
     protected Telemetry telemetry;
     protected int iterationCount = 1;
     protected ElapsedTime matchTimer;

     public Module(OpMode op){
          this.hardwareMap = op.hardwareMap;
          this.telemetry = op.telemetry;
     }
     public boolean isInizialized = true;
     public void setMatchTimer(ElapsedTime matchTimer) {
          this.matchTimer = matchTimer;
     }
     public void resetTimer() {
          this.matchTimer.reset();
     }
     public void increaseIteration() {
         iterationCount++;
     }
     public abstract void showData();
}
