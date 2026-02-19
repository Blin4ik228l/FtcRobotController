package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.AngleController;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.Collector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.FlyWheelClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;

public class HoodedShoter extends UpdatableModule {
    public CameraClass cameraClass;
    public AngleController angleController;
    public TurretMotor turretMotor;
    public FlyWheelClass flyWheelClass;
    public Collector collector;
    public DigitalCellsClass digitalCellsClass;

    public  HoodedShoter(OpMode op, VoltageSensorClass voltageSensorClass) {
        super(op);
        turretMotor = new TurretMotor(op);

        flyWheelClass = new FlyWheelClass(op);
        collector = new Collector(op);
        angleController = new AngleController(op);

        cameraClass = new CameraClass(op);

        telemetry.addLine(getClass().getName());
    }

    @Override
    public void update() {
        digitalCellsClass.update();
        turretMotor.turretOdometry.update();
    }

    @Override
    public void showData() {
        telemetry.addLine("===COLLECTOR DATA===");
        digitalCellsClass.showData();

    }
}
