package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;

public abstract class MotorModule extends Module {
    public VoltageSensorClass voltageSensorClass;
    public MotorModule(OpMode op) {
        super(op);
    }
    public void setVoltageSensorClass(VoltageSensorClass voltageSensorClass) {
        this.voltageSensorClass = voltageSensorClass;
    }
}
