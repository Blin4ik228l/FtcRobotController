package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.Delays;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.kPIDS;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.ServoPositions;


public abstract class Module implements  Delays, AnotherConsts, kPIDS, ServoPositions{
     protected HardwareMap hardwareMap;
     protected Telemetry telemetry;
     protected int iterationCount = 1;

     public Module(OpMode op){
          this.hardwareMap = op.hardwareMap;
          this.telemetry = op.telemetry;
     }
     public boolean isInitialized = true;
     public void increaseIteration() {
         iterationCount++;
     }
     public abstract void showData();
     public void sayModuleName(){
          telemetry.addLine( "===" + this.getClass().getSimpleName().toUpperCase() + "===");
     }
     public void sayInited(){
          if(!isInitialized) telemetry.addLine(this.getClass().getSimpleName() + " " + "CRUSHED");
          else telemetry.addLine(this.getClass().getSimpleName() + " " + "scfly Inited");
     }
}
