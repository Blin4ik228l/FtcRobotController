package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class VoltageSensorClass extends UpdatableCollector {
    private HardwareMap.DeviceMapping<VoltageSensor> voltageSensor;
    private double curVoltage;
    public VoltageSensorClass() {
        super(false);

        voltageSensor = MainFile.op.hardwareMap.voltageSensor;

        sayCreated();
    }

    public double getCurVoltage() {
        return curVoltage;
    }
    @Override
    protected void updateExt() {
        double result = Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor : voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }

        curVoltage = result;
    }

    @Override
    public void showDataExt() {
//        telemetry.addData("Hrdw size", hardwareMap.size());
        telemetry.addData("Vol", curVoltage);
    }
}