package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.MainModule;
import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ButtonClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.Servos;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.TeamColor;

public class RobotClass extends TeamColor {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */

    public RobotClass(OpMode op, String teamColor){
        super(teamColor, op);

        driveTrain = new MecanumDrivetrain(op, this);
        collector = new Collector(op);
        voltageSensor = new VoltageSensorClass(op);
    }
    public MecanumDrivetrain driveTrain;
    public Collector collector;
    public VoltageSensorClass voltageSensor;

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


            telemetry.addLine("Drivetrain Inited");
        }
        public DrivetrainMotors motors;
        public ExOdometry exOdometry;
        public CameraClass cameraClass;


        @Override
        public void update() {
            cameraClass.update();
            exOdometry.update();
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


            telemetry.addLine("Collector inited");
        }

        public CollectorMotors motors;
        public Servos servos;
        public ColorSensor colorSensor;
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


