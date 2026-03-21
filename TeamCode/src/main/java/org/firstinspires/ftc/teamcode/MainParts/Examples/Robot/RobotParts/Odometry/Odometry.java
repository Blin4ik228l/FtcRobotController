package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry;

import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.HoodedShooter;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position3D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector3;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class Odometry extends UpdatableCollector {
    //Все энкодеры на телеге + гироскоп + камера  составляющие общую систему оценки положения робота в пространстве.
    public final CameraClass cameraClass;
    private final DrivetrainMotors motors;
    private final GyroscopeClass gyro;
    private final TurretMotor turretMotor;
    private final OdometryData dataForRobot;
    private final OdometryData dataForTurret;
    public final OdometryBuffer bufferForRobot;
    public final OdometryBuffer bufferForTurret;

    private GeneralInformation generalInformation;
    public Odometry(MecanumDrivetrain drivetrain, HoodedShooter hoodedShooter, CameraClass cameraClass, GyroscopeClass gyro){
        super(false);
        generalInformation = MainFile.generalInformation;
        dataForRobot = new OdometryData();
        bufferForRobot = new OdometryBuffer();

        dataForTurret = new OdometryData();
        bufferForTurret = new OdometryBuffer();

        this.cameraClass = cameraClass;

        motors = drivetrain.motors;
        this.gyro = gyro;
        turretMotor = hoodedShooter.turretMotor;

        sayCreated();
    }
    public void setStartPos(OdometryData savedRobotData, OdometryData savedTurretData){
        dataForTurret.setPosition(savedRobotData.getPosition());
        dataForTurret.setPosition(savedTurretData.getPosition());
    }

    @Override
    protected void updateExt() {
        OdometryBuffer encodersBuf = motors.encodersBuffer;
        OdometryBuffer gyroBuf = gyro.gyroBuffer;
        OdometryBuffer turretBuf = turretMotor.turretBuffer;

        OdometryData gyroData = gyroBuf.read2();
        OdometryData encodersData = encodersBuf.read2();
        OdometryData turretData = turretBuf.read2();

        double headVelRobot = gyroData.getHeadVel() != 0 ? gyroData.getHeadVel() : encodersData.getHeadVel();
        double headAccelRobot = gyroData.getHeadAccel() != 0 ? gyroData.getHeadAccel() : encodersData.getHeadAccel();
        dataForRobot
                .setVelocity(encodersData.getVelocity())
                .setAccel(encodersData.getAccel())
                .setHeadVel(headVelRobot)
                .setHeadAccel(headAccelRobot);

        dataForTurret
                .setHeadVel(turretData.getHeadVel() + headVelRobot)
                .setHeadAccel(turretData.getHeadAccel() + headAccelRobot);

        //Камера видит таг -> Полностью считываем позицию с неё
        if (cameraClass.decisionMargin > 0 && Math.abs(dataForTurret.getHeadVel()) < MIN_TURRET_HEAD_SP){

            Position3D pos = cameraClass.absoluteRawPos3D;

            Vector3 matrix = new Vector3(pos.getX(), pos.getY(), pos.getZ()).rotate(Math.toRadians(-15), 0, Math.toRadians(-90) + turretMotor.localHead);

            double tagX = cameraClass.id == 20 ? generalInformation.generalObjects.blueTagCoord[0] : generalInformation.generalObjects.redTagCoord[0];
            double tagY = cameraClass.id == 20 ? generalInformation.generalObjects.blueTagCoord[1] : generalInformation.generalObjects.redTagCoord[1];

            Vector2 robotVector = new Vector2(matrix.getX(), matrix.getY()).rotateToGlobal(pos.getYaw());
            double robotX = tagX - robotVector.x;
            double robotY = tagY - robotVector.y;

            Position2D robotPos = new Position2D(robotX, robotY, pos.getYaw());

            dataForRobot.getPosition().setX(robotPos.getX());
            dataForRobot.getPosition().setY(robotPos.getY());
            dataForRobot.getPosition().setHeading(robotPos.getHeading() - (Math.toRadians(-90) + turretMotor.localHead));

            dataForTurret.getPosition().setX(robotPos.getX());
            dataForTurret.getPosition().setY(robotPos.getY());
            dataForTurret.getPosition().setHeading(robotPos.getHeading());
        }else{
            Position2D turretPos = turretData.getPosition();
            Position2D encodersPos = encodersData.getPosition();
            Position2D gyroPos = gyroData.getPosition();

            //Если гиро умер
            double deltaHead = gyroPos.getHeading() != 0 ? gyroPos.getHeading() : encodersPos.getHeading();

            //добавляем угол из буффера турели
            dataForTurret.getPosition().add(0,0, turretPos.getHeading() + deltaHead);

            //добавляем угол из буффера гироскопа
            dataForRobot.getPosition().add(0, 0, deltaHead);

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            Vector2 deltaVector2 = new Vector2(encodersPos.getX(), encodersPos.getY()).rotateToGlobal(dataForRobot.getPosition().getHeading());

            dataForRobot.getPosition().add(deltaVector2.x, deltaVector2.y , 0);
            dataForTurret.getPosition().add(deltaVector2.x, deltaVector2.y , 0);
        }

//        dataForRobot.rotateVelocity();
//        dataForTurret.rotateAccel();

        bufferForRobot.beginWrite().set(dataForRobot);
        bufferForRobot.endWrite();

        bufferForTurret.beginWrite().set(dataForTurret);
        bufferForTurret.endWrite();
    }
    @Override
    protected void showDataExt() {
        telemetry.addLine("RobotData");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", bufferForRobot.read().getPosition().getX(), bufferForRobot.read().getPosition().getY(), bufferForRobot.read().getPosition().getHeading() * RAD);
        telemetry.addData("Velocity", "X:%.1fcm/s Y:%.1fcm/s, Len: %.2f", bufferForRobot.read().getVelocity().x, bufferForRobot.read().getVelocity().y, bufferForRobot.read().getVelocity().length());
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", bufferForRobot.read().getHeadVel() * RAD, bufferForRobot.read().getHeadAccel() * RAD);
        telemetry.addLine("TuretData");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", bufferForTurret.read().getPosition().getX(), bufferForTurret.read().getPosition().getY(), bufferForTurret.read().getPosition().getHeading() * RAD);
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", bufferForTurret.read().getHeadVel() * RAD, bufferForTurret.read().getHeadAccel() * RAD);
        cameraClass.showData();
    }
}