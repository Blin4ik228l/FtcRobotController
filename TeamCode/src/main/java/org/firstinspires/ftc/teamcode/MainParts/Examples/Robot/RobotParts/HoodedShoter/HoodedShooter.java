package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.AngleController;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.Collector;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.FlyWheelClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class HoodedShooter extends UpdatableCollector {
    public AngleController angleController;
    public TurretMotor turretMotor;
    public FlyWheelClass flyWheelClass;
    public Collector collector;
    public DigitalCellsClass digitalCellsClass;

    public HoodedShooter() {
        super(true);
        turretMotor = new TurretMotor();
        flyWheelClass = new FlyWheelClass();
        collector = new Collector();

        angleController = new AngleController();

        digitalCellsClass = new DigitalCellsClass();

        sayCreated();
    }

    @Override
    protected void updateExt() {
        turretMotor.encodersClass.update(iterationCount, 1);
        flyWheelClass.encodersClass.update(iterationCount, 1);
        digitalCellsClass.update(iterationCount, 3);
    }

    @Override
    protected void showDataExt() {
        digitalCellsClass.showData();
        turretMotor.showData();
        collector.showData();
        flyWheelClass.showData();
        angleController.showData();
    }
}
