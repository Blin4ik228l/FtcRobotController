package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.Collector;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.MecanumDrivetrain;

public class RobotClass extends UpdatableModule{
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */
    public MecanumDrivetrain drivetrain;
    public Collector collector;


    public RobotClass(OpMode op){
        super(op);

        drivetrain = new MecanumDrivetrain(op);
        collector = new Collector(op);

        collector.motors.setPreferences(CollectorMotors.ControlMode.By_power, CollectorMotors.Units.Rad_in_sec);
    }

    @Override
    public void resetTimer() {
        innerRunTime.reset();

        drivetrain.resetTimer();
        collector.resetTimer();
    }

    @Override
    public void setIteration(int iteration) {
        super.setIteration(iteration);
        drivetrain.setIteration(iteration);
        collector.setIteration(iteration);
    }

    public void startTimer(){
        innerRunTime.reset();
    }

    @Override
    public void update() {
        drivetrain.update();

        if (iterationCount % 2 == 0) collector.update();
        if (iterationCount % 10 == 0) voltageSensor.update();

        collector.digitalCellsClass.setRandomizedArtifact(drivetrain.positionRobotController.getCameraClass().getRandomizedArtifacts());
    }

    @Override
    public void showData(){
        telemetry.addData("Match time:", innerRunTime.seconds());
        voltageSensor.showData();
        drivetrain.showData();
        collector.showData();
    }
}


