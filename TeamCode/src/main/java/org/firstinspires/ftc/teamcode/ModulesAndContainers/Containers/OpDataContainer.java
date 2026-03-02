package org.firstinspires.ftc.teamcode.ModulesAndContainers.Containers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MainModule;

public class OpDataContainer extends ModulesCollector{
    public UpdatableContainer updatableContainer;
    public ExecutableContainer executableContainer;
    public TelemetryContainer telemetryContainer;
    public Container[] containers;
    public OpDataContainer(MainFile mainFile){
        super(mainFile);
        updatableContainer = new UpdatableContainer(this);
        executableContainer = new ExecutableContainer(this);
        telemetryContainer = new TelemetryContainer(this);

        containers = new Container[]{
                updatableContainer,
                executableContainer,
                telemetryContainer
        };
    }
    public void start(){
        for (Container container : containers) {
            container.start();
        }
    }
    //Останавливаем потоки и записываем в файл
    public void interrupt(){
        for (Container container : containers) {
            container.interrupt();
        }

        fileSystem.stop();
    }
}
