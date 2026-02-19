package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.HoodedShoter;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;

public class RobotClass extends UpdatableModule{
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */
    private VoltageSensorClass voltageSensor;

    public MecanumDrivetrain drivetrain;
    public HoodedShoter hoodedShoter;

    public Odometry odometry;

    public RobotClass(OpMode op){
        super(op);
        voltageSensor = new VoltageSensorClass(op);

        drivetrain = new MecanumDrivetrain(op, voltageSensor);
        hoodedShoter = new HoodedShoter(op, voltageSensor);

        odometry = new Odometry(op, drivetrain, hoodedShoter);
    }

    @Override
    public void update() {
        voltageSensor.update();
        odometry.update();
    }

    @Override
    public void showData(){
        telemetry.addData("Match time:", matchTimer.seconds());
        voltageSensor.showData();
        drivetrain.showData();
        hoodedShoter.showData();
    }
}


