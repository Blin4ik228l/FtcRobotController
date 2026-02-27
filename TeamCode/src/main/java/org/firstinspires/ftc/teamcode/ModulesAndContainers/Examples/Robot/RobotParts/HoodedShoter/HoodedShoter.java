package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.AngleController;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.Collector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.FlyWheelClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;

public class HoodedShoter extends UpdatableModule {
    public AngleController angleController;
    public TurretMotor turretMotor;
    public FlyWheelClass flyWheelClass;
    public Collector collector;
    public DigitalCellsClass digitalCellsClass;

    public HoodedShoter(OpMode op, VoltageSensorClass voltageSensorClass) {
        super(op);
        turretMotor = new TurretMotor(op, voltageSensorClass);
        flyWheelClass = new FlyWheelClass(op, voltageSensorClass);
        collector = new Collector(op, voltageSensorClass);

        angleController = new AngleController(op);

        digitalCellsClass = new DigitalCellsClass(op);

        sayInited();
    }

    @Override
    protected void update() {
        digitalCellsClass.update();
    }

    @Override
    protected void showData() {
        telemetry.addLine("===COLLECTOR DATA===");
        digitalCellsClass.safeShowData();
        turretMotor.safeShowData();
        collector.safeShowData();
        flyWheelClass.safeShowData();
        telemetry.addLine();
    }
}
