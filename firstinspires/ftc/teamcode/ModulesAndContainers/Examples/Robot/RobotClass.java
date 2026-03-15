package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Collector;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.CollectorMotors;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.VoltageSensorClass;

public class RobotClass extends UpdatableModule{
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */
    public VoltageSensorClass voltageSensor;
    public MecanumDrivetrain drivetrain;
    public Collector collector;

    public RobotClass(OpMode op){
        super(op);

        try {
            voltageSensor = new VoltageSensorClass(op);
        } catch (Exception e) {
            voltageSensor.isInizialized = false;
        }

        drivetrain = new MecanumDrivetrain(op);
        collector = new Collector(op);

        collector.motors.setPreferences(CollectorMotors.ControlMode.By_power, CollectorMotors.Units.Rad_in_sec);

        collector.motors.voltageSensorClass = voltageSensor;
        drivetrain.motors.voltageSensorClass = voltageSensor;
    }

    @Override
    public void update() {
        drivetrain.update();

        if (iterationCount % 2 == 0) collector.update();
        if (iterationCount % 10 == 0) voltageSensor.update();
    }

    @Override
    public void showData(){
        telemetry.addData("Match time:", matchTimer.seconds());
        voltageSensor.showData();
        drivetrain.showData();
        collector.showData();
    }
}


