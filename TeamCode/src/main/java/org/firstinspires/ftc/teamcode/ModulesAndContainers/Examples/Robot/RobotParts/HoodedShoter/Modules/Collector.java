package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutableCollector;

public class Collector extends ExecutableCollector {
    public String collector = controlHubDevices.getMotor(3);

    public Collector(MainFile mainFile) {
        super(mainFile);
        createMotorWrapperUtils();
        motorsCollector.add(motorBuilder.initialize(mainFile, collector).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(12.5, 1.0).get());

        sayCreated();
    }

    @Override
    protected void executeExt(Double... args) {
        motorsCollector.get(collector).execute(args);
    }

    @Override
    protected void showDataExt() {

    }

}
