package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableModule;

public class VoltageSensorClass extends UpdatableModule {
    private double curVoltage;
    private double MAX_VOL;
    private double kPower;

    public VoltageSensorClass(OpMode op) {
        super(op);
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
    public void update() {
        double result = Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
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
    public void showData() {
        telemetry.addLine("===VoltageClass===");
        telemetry.addData("H size", hardwareMap.size());
        telemetry.addData("Vol", curVoltage);
        telemetry.addData("Max vol", MAX_VOL);
        telemetry.addData("KP", kPower);
        telemetry.addLine();
    }
}