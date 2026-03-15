package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.ExecutableCollector;

public class Collector extends ExecutableCollector {
    public String collector = controlHubDevices.getMotor(3);

    public Collector() {
        super(false);
        createMotorWrapperUtils();
        motorsCollector.add(motorBuilder.initialize(collector).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(12.5, 1.0, 0.0, 3.0, 1.0).get());

        sayCreated();
    }

    @Override
    protected void executeExt(Double... args) {
        motorsCollector.get(collector).execute(args);
    }

    @Override
    protected void showDataExt() {
        motorsCollector.get(collector).showData();
    }
}
