package org.firstinspires.ftc.teamcode.MainParts.Containers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

public class OpDataContainer extends ModulesCollector{
    public UpdatableContainer updatableContainer;
    public ExecutableContainer executableContainer;
    public TelemetryContainer telemetryContainer;
    public Container[] containers;
    public OpDataContainer(GeneralInformation generalInformation, OpMode op){
        super(generalInformation, op);
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
