package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.AngleController;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.Collector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.FlyWheelClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

public class HoodedShoter extends UpdatingModule {
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

        setUpdateCount(3);
        sayCreated();
    }

    @Override
    protected void updateExt() {
        digitalCellsClass.update();
    }

    @Override
    protected void showDataExt() {
        digitalCellsClass.showData();
        turretMotor.showData();
        collector.showData();
        flyWheelClass.showData();
    }
}
