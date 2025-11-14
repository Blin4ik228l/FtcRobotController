package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Module;

public abstract class TeleOpModernized extends OpMode {
    public ExecuteModule moduleCamera, modulePlayer1, moduleAutomaticClass, moduleMotorsController, moduleColorSensor, moduleJoystickActivity, moduleInnerWarden;
    public void startExecute() {
        moduleJoystickActivity.start();

        moduleCamera.start();

        modulePlayer1.start();

        moduleAutomaticClass.start();
        moduleMotorsController.start();
        moduleColorSensor.start();

        moduleInnerWarden.start();
    }
    public void interruptAll(){
        moduleJoystickActivity.interrupt();

        moduleCamera.interrupt();

        modulePlayer1.interrupt();

        moduleAutomaticClass.interrupt();
        moduleMotorsController.interrupt();
        moduleColorSensor.interrupt();

        moduleInnerWarden.interrupt();
    }
  public static class ExecuteModule extends Thread{
        public Module module;
        public ExecuteModule(Module module){
            this.module = module;
        }

        @Override
        public void run() {
            while (!isInterrupted()){
                module.execute();
            }
        }
    }
}

