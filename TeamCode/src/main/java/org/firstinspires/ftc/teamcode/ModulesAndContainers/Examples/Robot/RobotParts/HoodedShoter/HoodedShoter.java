package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.AngleController;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.Collector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.FlyWheelClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MainModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableCollector;

public class HoodedShoter extends MainModule {
    public AngleController angleController;
    public TurretMotor turretMotor;
    public FlyWheelClass flyWheelClass;
    public Collector collector;
    public DigitalCellsClass digitalCellsClass;

    public HoodedShoter(MainFile mainFile) {
        super(mainFile);
        turretMotor = new TurretMotor(mainFile);
        flyWheelClass = new FlyWheelClass(mainFile);
        collector = new Collector(mainFile);

        angleController = new AngleController(mainFile);

        digitalCellsClass = new DigitalCellsClass(mainFile);

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
