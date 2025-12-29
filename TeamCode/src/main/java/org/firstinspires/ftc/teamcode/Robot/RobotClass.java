package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.Collector;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.VoltageSensorClass;

public class RobotClass extends UpdatableModule {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */
    public MecanumDrivetrain driveTrain;
    public Collector collector;
    public VoltageSensorClass voltageSensor;
    public ElapsedTime innerRunTime;
    private int iterationCount;

    public RobotClass(TeamClass.Color color, TeamClass.StartPos startPos, OpMode op ){
        super(op.telemetry);

        driveTrain = new MecanumDrivetrain(color, startPos, op);
        collector = new Collector(op);
        voltageSensor = new VoltageSensorClass(op);

        innerRunTime = new ElapsedTime();
    }

    public void setIterationCount(int iterationCount) {
        this.iterationCount = iterationCount;
    }

    public void startTimer(){
        innerRunTime.reset();
    }

    @Override
    public void update() {
        driveTrain.update();

        if (iterationCount % 2 == 0) collector.update();
        if (iterationCount % 10 == 0) voltageSensor.update();
    }

    @Override
    public void showData(){
        telemetry.addData("Match time:", innerRunTime.seconds());
        voltageSensor.showData();
        driveTrain.showData();
        collector.showData();
    }
}


