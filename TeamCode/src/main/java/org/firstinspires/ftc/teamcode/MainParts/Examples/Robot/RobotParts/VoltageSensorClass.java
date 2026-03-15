package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class VoltageSensorClass extends UpdatableCollector {
    private HardwareMap.DeviceMapping<VoltageSensor> voltageSensor;
    private double curVoltage;
    private double MAX_VOL;
    private double kPower;
    public VoltageSensorClass() {
        super(false);

        voltageSensor = MainFile.op.hardwareMap.voltageSensor;

        sayCreated();
    }
    public double getkPower() {
        return kPower;
    }

    public double getCurVoltage() {
        return curVoltage;
    }

    public double getMAX_VOL() {
        return MAX_VOL;
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
        if(curVoltage > MAX_VOL) MAX_VOL = curVoltage;

        kPower = MAX_VOL / curVoltage;
    }

    @Override
    public void showDataExt() {
//        telemetry.addData("Hrdw size", hardwareMap.size());
        telemetry.addData("Vol", curVoltage);
    }
}