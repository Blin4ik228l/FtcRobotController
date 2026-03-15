package org.firstinspires.ftc.teamcode.MainParts.Containers;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Module;

public class Container implements Runnable{
    protected Thread thread;
    protected ModulesCollector modulesCollector;
    protected Module[] allModules;

    protected volatile boolean running = true;
    public Container(ModulesCollector modulesCollector) {
       this.modulesCollector = modulesCollector;
       this.allModules = modulesCollector.modules;

       this.thread = new Thread(this);
    }

    @Override
    public void run() {

    }
    public void start(){
        thread.start();
    }
    public void interrupt(){
        running = false;
        thread.interrupt();
    }
}
