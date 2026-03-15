package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Module;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.VoltageSensorClass;

public abstract class MotorModule extends Module {
    public VoltageSensorClass voltageSensorClass;
    public MotorModule(OpMode op) {
        super(op);
    }
    public void setVoltageSensorClass(VoltageSensorClass voltageSensorClass) {
        this.voltageSensorClass = voltageSensorClass;
    }
}
