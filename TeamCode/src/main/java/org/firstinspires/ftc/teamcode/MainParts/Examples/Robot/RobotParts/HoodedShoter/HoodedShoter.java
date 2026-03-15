package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.AngleController;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.Collector;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.FlyWheelClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.MainModule;

public class HoodedShoter extends MainModule {
    public AngleController angleController;
    public TurretMotor turretMotor;
    public FlyWheelClass flyWheelClass;
    public Collector collector;
    public DigitalCellsClass digitalCellsClass;

    public HoodedShoter() {
        turretMotor = new TurretMotor();
        flyWheelClass = new FlyWheelClass();
        collector = new Collector();

        angleController = new AngleController();

        digitalCellsClass = new DigitalCellsClass();

        sayCreated();
    }
    @Override
    protected void showDataExt() {
        digitalCellsClass.showData();
//        turretMotor.showData();
//        collector.showData();
        flyWheelClass.showData();
        angleController.showData();
    }

}
