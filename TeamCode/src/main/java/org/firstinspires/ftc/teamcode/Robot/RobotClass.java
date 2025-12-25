package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ButtonClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ColorSensorClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.ServomotorsClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.PositionRobotController;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.TeamColorClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.VoltageSensorClass;

public class RobotClass extends UpdatableModule {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */

    public RobotClass(TeamColorClass.Color color, TeamColorClass.StartPos startPos, OpMode op ){
        super(op.telemetry);

        driveTrain = new MecanumDrivetrain(color, startPos, op);
        collector = new Collector(op);
        voltageSensor = new VoltageSensorClass(op);

        innerTime = new ElapsedTime();
    }

    public MecanumDrivetrain driveTrain;
    public Collector collector;
    public VoltageSensorClass voltageSensor;
    public ElapsedTime innerTime;

    public void start(){
        innerTime.reset();
    }

    @Override
    public void update() {
        driveTrain.update();
        collector.update();
        voltageSensor.update();
    }

    @Override
    public void showData(){
        telemetry.addData("Match time:", innerTime.seconds());
        voltageSensor.showData();
        driveTrain.showData();
        collector.showData();
    }

    public static class MecanumDrivetrain extends UpdatableModule {
        //Телега робота(моторы + колёса) с энкодерами, гироскопом и камерой.

        public MecanumDrivetrain(TeamColorClass.Color color, TeamColorClass.StartPos startPos, OpMode op){
            super(op.telemetry);
            teamColorClass = new TeamColorClass(color, startPos, op);

            motors = new DrivetrainMotors(op);

            odometryClass = new OdometryClass(op);
            cameraClass = new CameraClass(op, teamColorClass);

            positionRobotController = new PositionRobotController(odometryClass, teamColorClass, cameraClass, op);

            telemetry.addLine("Drivetrain Inited");
        }
        public TeamColorClass teamColorClass;
        public DrivetrainMotors motors;
        public OdometryClass odometryClass;
        public CameraClass cameraClass;
        public PositionRobotController positionRobotController;

        @Override
        public void update() {
            positionRobotController.update();
        }

        @Override
        public void showData() {
            teamColorClass.showData();
            odometryClass.showData();
            cameraClass.showData();
            positionRobotController.showData();
            motors.showData();
        }
    }

    public static class Collector extends UpdatableModule {
        public Collector(OpMode op) {
            super(op.telemetry);

            motors = new CollectorMotors(op);

            servos = new ServomotorsClass(op);
            colorSensorClass = new ColorSensorClass(op);
            buttonClass = new ButtonClass(op);

            digitalCellsClass = new DigitalCellsClass(servos, op);

            telemetry.addLine("Collector inited");
        }

        public CollectorMotors motors;
        public ServomotorsClass servos;
        public ColorSensorClass colorSensorClass;
        public DigitalCellsClass digitalCellsClass;
        public ButtonClass buttonClass;

       @Override
       public void update() {
//           buttonClass.update();
           colorSensorClass.update();
       }

       @Override
       public void showData() {
           buttonClass.showData();
           digitalCellsClass.showData();
//           colorSensorClass.showData();
           servos.showData();
           motors.showData();
       }
   }
}


