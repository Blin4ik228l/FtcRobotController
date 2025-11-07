package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticParts.EncodersInMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticParts.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticParts.Servos;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.MotorsOnDrivetrain;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticParts.MotorsOnCollector;
import org.firstinspires.ftc.teamcode.TeamColor;

public class RobotClass extends TeamColor {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */

    public RobotClass(OpMode op, String teamColor){
        super(teamColor);

        driveTrain = new MecanumDrivetrain(op, this);
        collector = new Collector(op);
    }
    public static MecanumDrivetrain driveTrain;
    public static Collector collector;

    public static class MecanumDrivetrain extends Module {
        //Телега робота(моторы + колёса) с энкодерами, гироскопом и камерой.

        public MecanumDrivetrain(OpMode op, TeamColor teamColor){
            super(op.telemetry);

            motors = new MotorsOnDrivetrain(op);

            exOdometry = new ExOdometry(op,teamColor );

            telemetry.addLine("Drivetrain Inited");
        }
        public MotorsOnDrivetrain motors;
        public ExOdometry exOdometry;
        public void setPower(double yVol, double xVol, double angVol){
            if(Math.abs(yVol) < 0.10) yVol = 0.10 * Math.signum(yVol);
            if(Math.abs(xVol) < 0.10) xVol = 0.10 * Math.signum(xVol);
            if(Math.abs(angVol) < 0.10) angVol = 0.10 * Math.signum(angVol);

            yVol *= 1;
            xVol *= 1.1;
            angVol *= 1.3;

            //движение по y - это вперёд - назад
            //движение по x - это влево - вправо
            //angVol поворот
            exOdometry.updateAll();//Обноволяем одометрию постоянно когда вызываем метод setPower

            motors.getLeftF().setPower( yVol - xVol - angVol);
            motors.getLeftB().setPower( yVol + xVol - angVol);
            motors.getRightF().setPower(yVol + xVol + angVol);
            motors.getRightB().setPower(yVol - xVol + angVol);
        }
    }

    public static class Collector extends Module{
        public Collector(OpMode op) {
            super(op.telemetry);

            motors = new MotorsOnCollector(op);
            encoders = new EncodersInMotors(op);

            servos = new Servos(op);
            colorSensor = new ColorSensor(op);

            automaticClass = new AutomaticClass(op, this);
            telemetry.addLine("Collector inited");
        }
        public MotorsOnCollector motors;
        public Servos servos;
        public ColorSensor colorSensor;
        public EncodersInMotors encoders;
        boolean isStartFiring = false;
        boolean isFullyLoaded = false;
        public AutomaticClass automaticClass;

        public void setAll(boolean isTurnOn, boolean isFlyWheelOn, double velocity){
            automaticClass.setAll(isTurnOn, isFlyWheelOn, velocity);
        }
        public void showSizeAndPos(){
//            telemetry.addData("Size",servos.selfMath.loadedArtifacts.size());
//            if(isFullyLoaded){
//            telemetry.addLine("Pos")
//                    .addData("Pos/Color", servos.selfData.loadedArtifacts.get(0).pos + " " + servos.selfData.loadedArtifacts.get(0).color)
//                    .addData("Pos/Color", servos.selfData.loadedArtifacts.get(1).pos + " " + servos.selfData.loadedArtifacts.get(1).color)
//                    .addData("Pos/Color", servos.selfData.loadedArtifacts.get(2).pos + " " + servos.selfData.loadedArtifacts.get(2).color);
//            }
        }
    }
}

