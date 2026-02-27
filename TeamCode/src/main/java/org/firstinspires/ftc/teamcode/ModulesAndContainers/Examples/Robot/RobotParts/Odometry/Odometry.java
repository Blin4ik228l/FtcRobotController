package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.CameraClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.HoodedShoter;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;

public class Odometry extends UpdatableModule{
    //Все энкодеры на телеге + гироскоп + камера  составляющие общую систему оценки положения робота в пространстве.
    public CameraClass cameraClass;
    private DrivetrainMotors.EncoderClass encoders;
    private GyroscopeClass gyro;
    private TurretMotor.TurretOdometry turretOdometry;

    private OdometryData outPutDataForRobot;

    public OdometryBuffer odometryBufferForRobot;
    private OdometryData outPutDataForTuret;

    public OdometryBuffer odometryBufferForTuret;

    public Odometry(OpMode op, MecanumDrivetrain drivetrain, HoodedShoter hoodedShoter){
        super(op);
        outPutDataForRobot = new OdometryData();
        odometryBufferForRobot = new OdometryBuffer();

        outPutDataForTuret = new OdometryData();
        odometryBufferForTuret = new OdometryBuffer();

        cameraClass = new CameraClass(op);

        encoders = drivetrain.motors.encoderClass;
        gyro = drivetrain.gyro;
        turretOdometry = hoodedShoter.turretMotor.turretOdometry;

        sayInited();
    }
    public void setStartPos(Position2D position2D){
        outPutDataForTuret.setPosition(position2D);
    }

    @Override
    public void update(){
        cameraClass.update();
        turretOdometry.update();
        gyro.update();

        OdometryBuffer encodersBuf = encoders.encodersBuffer;
        OdometryBuffer gyroBuf = gyro.gyroBuffer;
        OdometryBuffer turretBuf = turretOdometry.turretBuffer;

        outPutDataForRobot
                .setVelocity(encodersBuf.read().getVelocity())
                .setAccel(encodersBuf.read().getAccel())
                .setHeadVel(gyroBuf.read().getHeadVel())
                .setHeadAccel(gyroBuf.read().getHeadAccel());

        outPutDataForTuret
                .setHeadVel(turretBuf.read().getHeadVel())
                .setHeadAccel(turretBuf.read().getHeadVel());

        //Камера видит таг -> Полностью считываем позицию с неё
        if (cameraClass.absoluteData.getDesisionMarg() > 0){
            Position2D pos = cameraClass.absoluteData.getPosition();

            outPutDataForRobot.getPosition().setX(pos.getX());
            outPutDataForRobot.getPosition().setY(pos.getY());
            outPutDataForRobot.getPosition().setHeading(pos.getHeading() - turretOdometry.localHead);

            outPutDataForTuret.getPosition().setX(pos.getX());
            outPutDataForTuret.getPosition().setY(pos.getY());
            outPutDataForTuret.getPosition().setHeading(pos.getHeading());
        }else{
            Position2D turretPos = turretBuf.read2().getPosition();
            Position2D encodersPos = encodersBuf.read2().getPosition();
            Position2D gyroPos = gyroBuf.read2().getPosition();

            //добавляем угол из буффера турели
            outPutDataForTuret.getPosition().add(0,0, turretPos.getHeading() + gyroPos.getHeading());

            //добавляем угол из буффера гироскопа
            outPutDataForRobot.getPosition().add(0, 0, gyroPos.getHeading());

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            Vector2 vector2 = new Vector2(encodersPos.getX(), encodersPos.getY()).rotateToGlobal(outPutDataForRobot.getPosition().getHeading());

            outPutDataForRobot.getPosition().add(vector2.x, vector2.y , 0);
            outPutDataForTuret.getPosition().add(vector2.x, vector2.y , 0);
        }

        outPutDataForRobot.rotateVelocity();
        outPutDataForRobot.rotateAccel();

        odometryBufferForRobot.beginWrite().set(outPutDataForRobot);
        odometryBufferForRobot.endWrite();

        odometryBufferForTuret.beginWrite().set(outPutDataForTuret);
        odometryBufferForTuret.endWrite();
    }

    @Override
    public void showData(){
        sayModuleName();
        telemetry.addLine("RobotData");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", odometryBufferForRobot.read().getPosition().getX(), odometryBufferForRobot.read().getPosition().getY(), odometryBufferForRobot.read().getPosition().getHeading() * RAD);
        telemetry.addData("Velocity", "X:%.1fcm/s Y:%.1fcm/s, Len: %.2f", odometryBufferForRobot.read().getVelocity().x, odometryBufferForRobot.read().getVelocity().y, odometryBufferForRobot.read().getVelocity().length());
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", odometryBufferForRobot.read().getHeadVel() * RAD, odometryBufferForRobot.read().getHeadAccel() * RAD);
        telemetry.addLine("TuretData");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", odometryBufferForTuret.read().getPosition().getX(), odometryBufferForTuret.read().getPosition().getY(), odometryBufferForTuret.read().getPosition().getHeading() * RAD);
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", odometryBufferForTuret.read().getHeadVel() * RAD, odometryBufferForTuret.read().getHeadAccel() * RAD);
        telemetry.addLine();

        gyro.showData();
        cameraClass.showData();
    }


}