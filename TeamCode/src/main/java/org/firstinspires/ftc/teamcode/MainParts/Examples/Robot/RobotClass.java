package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.HoodedShoter;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class RobotClass extends UpdatableCollector {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */
    private final VoltageSensorClass voltageSensor;
    public MecanumDrivetrain drivetrain;
    public HoodedShoter hoodedShoter;

    public Odometry odometry;
    public RobotClass(){
        voltageSensor = new VoltageSensorClass();
        MainFile.voltageSensorClass = voltageSensor;

        drivetrain = new MecanumDrivetrain();
        hoodedShoter = new HoodedShoter();

        odometry = new Odometry(drivetrain, hoodedShoter);

        sayCreated();
    }

    @Override
    protected void updateExt() {
        voltageSensor.update(iterationCount, 10);
        odometry.update(iterationCount, 1);
    }

    @Override
    protected void showDataExt() {
        voltageSensor.showData();
        odometry.showData();
    }
}


