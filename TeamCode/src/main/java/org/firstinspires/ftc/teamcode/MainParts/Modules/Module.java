package org.firstinspires.ftc.teamcode.MainParts.Modules;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.SomeConsts;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.Theory;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;


public abstract class Module implements Theory, SomeConsts {
     protected Telemetry telemetry;
     protected boolean isInitialized = true;
     public Module(){
          telemetry = MainFile.op.telemetry;
     }
     private void sayLoading(){
          telemetry.addLine("Now initialize" +  " " + this.getClass().getSimpleName().toUpperCase() + " " + "module");
     }

     protected void sayCreated(){
          telemetry.addLine(this.getClass().getSimpleName().toUpperCase() + " " + "obgect created");
     }

     public abstract void sayModuleName();
     protected abstract void showDataExt();
     protected abstract void sayLastWords();
     public void showData(){
          sayModuleName();
          showDataExt();
          sayLastWords();
     };
}
