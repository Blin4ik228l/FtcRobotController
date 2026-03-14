package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.ExecutorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.HoodedShoter;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;

public class RobotClass extends ExecutorModule {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */
    private VoltageSensorClass voltageSensor;
    public MecanumDrivetrain drivetrain;
    public HoodedShoter hoodedShoter;

    public Odometry odometry;
    public RobotClass(MainFile mainFile){
        super(mainFile);
        voltageSensor = new VoltageSensorClass(mainFile);
        mainFile.setVoltageSensorClass(voltageSensor);

        drivetrain = new MecanumDrivetrain(mainFile);
        hoodedShoter = new HoodedShoter(mainFile);

        odometry = new Odometry(mainFile, drivetrain, hoodedShoter);

        updateTime = new ElapsedTime();
        sayCreated();
    }


    @Override
    protected void executeExt() {
        if (iterationCount % 10 == 0)voltageSensor.update();
        odometry.update();
    }

    @Override
    public void executeTeleOp() {

    }

    @Override
    public void executeAuto() {

    }

    @Override
    protected void showDataExt() {
        voltageSensor.showData();
        odometry.showData();
    }
}


