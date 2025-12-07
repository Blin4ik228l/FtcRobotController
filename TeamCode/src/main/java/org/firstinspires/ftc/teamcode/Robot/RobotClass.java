package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ButtonClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.DigitalCells;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.Servos;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.PositionRobotController;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.VoltageSensorClass;

public class RobotClass extends TeamColor {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */

    public RobotClass(Color color, StartPos startPos, OpMode op  ){
        super(color, startPos, op);

        driveTrain = new MecanumDrivetrain(op, this);
        collector = new Collector(op);
        voltageSensor = new VoltageSensorClass(op);

        startTime = new ElapsedTime();
    }
    public MecanumDrivetrain driveTrain;
    public Collector collector;
    public VoltageSensorClass voltageSensor;
    public ElapsedTime startTime;

    public void start(){
        startTime.reset();

        driveTrain.cameraClass.generalLogic = CameraClass.GeneralLogic.Button_play_pressed;
    }

    @Override
    public void update() {
        driveTrain.update();
        collector.update();
        voltageSensor.update();
    }

    @Override
    public void showData(){
        driveTrain.showData();
        collector.showData();
        voltageSensor.showData();
    }

    public static class MecanumDrivetrain extends UpdatableModule {
        //Телега робота(моторы + колёса) с энкодерами, гироскопом и камерой.

        public MecanumDrivetrain(OpMode op, TeamColor teamColor){
            super(op.telemetry);

            motors = new DrivetrainMotors(op);

            exOdometry = new ExOdometry(op,teamColor );
            cameraClass = new CameraClass(op, teamColor);

            positionRobotController = new PositionRobotController(exOdometry, cameraClass, op);

            telemetry.addLine("Drivetrain Inited");
        }
        public DrivetrainMotors motors;
        public ExOdometry exOdometry;
        public CameraClass cameraClass;
        public PositionRobotController positionRobotController;

        @Override
        public void update() {
            positionRobotController.update();
        }

        @Override
        public void showData() {
            cameraClass.showData();
            exOdometry.showData();
            motors.showData();
        }
    }

    public static class Collector extends UpdatableModule {
        public Collector(OpMode op) {
            super(op.telemetry);

            motors = new CollectorMotors(op);

            servos = new Servos(op);
            colorSensor = new ColorSensor(op);
            buttonClass = new ButtonClass(op);

            digitalCells = new DigitalCells(servos, op);

            telemetry.addLine("Collector inited");
        }

        public CollectorMotors motors;
        public Servos servos;
        public ColorSensor colorSensor;
        public DigitalCells digitalCells;
        public ButtonClass buttonClass;

       @Override
       public void update() {
           colorSensor.update();
       }

       @Override
       public void showData() {
           servos.showData();
           motors.showData();
           colorSensor.showData();
           buttonClass.showData();
       }
   }

}


