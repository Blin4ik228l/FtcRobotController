package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

import java.util.ArrayList;

public class VoltageSensorClass extends UpdatableModule {
    private HardwareMap.DeviceMapping<VoltageSensor> voltageSensor;
    private double curVoltage;
    private double MAX_VOL;
    private double kPower;
    public VoltageSensorClass(MainFile mainFile) {
        super(mainFile, " ");

        try {
            voltageSensor = hardwareMap.voltageSensor;
        }catch (Exception e){
            isInitialized = false;
        }

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
        sayModuleName();
        telemetry.addData("H size", hardwareMap.size());
        telemetry.addData("Vol", curVoltage);
        telemetry.addData("Max vol", MAX_VOL);
        telemetry.addData("KP", kPower);
        telemetry.addLine();
    }
}