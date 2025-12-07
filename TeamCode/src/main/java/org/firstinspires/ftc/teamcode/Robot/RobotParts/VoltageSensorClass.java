package org.firstinspires.ftc.teamcode.Robot.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;

public class VoltageSensorClass extends UpdatableModule {
    public VoltageSensorClass(OpMode op){
        super(op.telemetry);
        hardwareMap = op.hardwareMap;
    }

    private HardwareMap hardwareMap;
    public double curVoltage;
    public double MAX_VOL = 0.0;
    public double kPower;
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

        kPower = curVoltage / MAX_VOL;
    }

    @Override
    public void showData() {
        telemetry.addLine("===VoltageClass===");
        telemetry.addData("vol", curVoltage);
        telemetry.addData("koef Power", kPower);
        telemetry.addLine();
    }
}
