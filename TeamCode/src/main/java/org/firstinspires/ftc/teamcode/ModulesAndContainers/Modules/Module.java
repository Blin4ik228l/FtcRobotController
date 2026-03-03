package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.Delays;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.kPIDS;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.ServoPositions;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;


public abstract class Module implements Delays, AnotherConsts, kPIDS, ServoPositions{
     protected Telemetry telemetry;
     protected boolean isInitialized = true;
     public Module(MainFile mainFile){
          telemetry = mainFile.op.telemetry;
     }
     public boolean isInterrupted;

     private void sayLoading(){
          telemetry.addLine("Now initialize" +  " " + this.getClass().getSimpleName().toUpperCase() + " " + "module");
     }

     protected void sayCreated(){
          telemetry.addLine(this.getClass().getSimpleName().toUpperCase() + " " + "obgect created");
     }

     public abstract void sayModuleName();
     protected abstract void showDataExt();
     public void showData(){
          sayModuleName();
          showDataExt();
     };
}
