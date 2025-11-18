package org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.EncoderClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.TeamColor;

public class ExOdometry extends UpdatableModule {
    //Все энкодеры на телеге + гироскоп + камера  составляющие общую систему оценки положения робота в пространстве.
    public ExOdometry(OpMode op, TeamColor teamColor){
        super(op.telemetry);
//            voltageSensor = op.hardwareMap.get(VoltageSensor.class, "");
        gyro = new GyroscopeClass(op);
        encoderClass = new EncoderClass(op);
        this.teamColor = teamColor;

        selfMath = new SelfMath();
        selfMath.deltaTimes = new double[3];
        selfMath.oldTimes = new double[3];

        encGlobalPosition2D = new Position2D();
        gyroGlobalPosition2D = new Position2D();
        detectedPos = new Position2D();

        robotCurVelocity = new Vector2();
        robotCurAccel = new Vector2();

        telemetry.addLine("ExOdometry Inited");
    }
    private GyroscopeClass gyro;
    public TeamColor teamColor;
    public EncoderClass encoderClass;
    public final SelfMath selfMath;
    public final Position2D gyroGlobalPosition2D;                                   // Относительное перемещение
    public final Position2D encGlobalPosition2D;
    public Vector2 robotCurVelocity;
    public Vector2 robotCurAccel;
    public double encHeadVel, encHeadAccel;
    public double gyroHeadVel, gyroHeadAccel;
    public Position2D detectedPos;
    public boolean isPosFromCameraWasGotFirstly = false;
    private double ticksToCm(double ticks){
        return ticks / encoderClass.COUNTS_PER_CM;
    }

    public void setPosFromCamera(Position2D cameraPos){
        if(!cameraPos.equals(new Position2D()) && !isPosFromCameraWasGotFirstly){


            isPosFromCameraWasGotFirstly = true;
        }
        detectedPos = cameraPos;
        encGlobalPosition2D.setX(detectedPos.getX());
        encGlobalPosition2D.setY(detectedPos.getY());
        encGlobalPosition2D.setHeading(detectedPos.getHeading());
//        if(isPosFromCameraWasGotFirstly && !cameraPos.equals(new Position2D())){
//            encGlobalPosition2D.setX(encGlobalPosition2D.getX() * 0.9 + cameraPos.getX() * 0.1);
//            encGlobalPosition2D.setY(encGlobalPosition2D.getY() * 0.9 + cameraPos.getY() * 0.1);
//            encGlobalPosition2D.setHeading(encGlobalPosition2D.getHeading() * 0.9 + cameraPos.getHeading() * 0.1);
//        }
    }
    public boolean isVyrCompleted;
    @Override
    public void update(){
        selfMath.calculateAll();
    }
    public double getFoundedRobotAngle(){
        double targX = teamColor.getWallCoord()[0] - encGlobalPosition2D.getX();
        double targY = teamColor.getWallCoord()[1] - encGlobalPosition2D.getY();

        return  (Math.atan2(targY, -targX) - Math.PI/2);
    }

    public double getDeltaAngle(double targetAngle){
        double target = (Math.signum(encGlobalPosition2D.getHeading()) * targetAngle + encGlobalPosition2D.getHeading()) % Math.PI;

        if(Math.abs(target) < Math.toRadians(1)) {
            target = 0;
            isVyrCompleted = true;
        }else {
            isVyrCompleted = false;
        }

        return target;
    }

    public double getRange(){
        return new Vector2(teamColor.getWallCoord()[0] - encGlobalPosition2D.getX(), teamColor.getWallCoord()[1] - encGlobalPosition2D.getY()).length();
    }
    public Position2D getDeltaPos(){
        Position2D targPos = new Position2D(teamColor.artifactsUnderBlueWallCoord[0][0], teamColor.artifactsUnderBlueWallCoord[0][1], teamColor.artifactsUnderBlueWallCoord[0][3]);

        return new Position2D(targPos.getX() - encGlobalPosition2D.getX(), targPos.getY() - encGlobalPosition2D.getY(), targPos.getHeading() - encGlobalPosition2D.getHeading());
    }

    @Override
    public void showData(){
        telemetry.addLine("=== EXODOMETRY ===");
        telemetry.addData("Position from encoders", "X:%.1f Y:%.1f H:%.1f°", encGlobalPosition2D.getX(), encGlobalPosition2D.getY(), detectedPos.getHeading() * 180/Math.PI);
        telemetry.addData("Position from gyro", "X:%.1f Y:%.1f H:%.1f°", gyroGlobalPosition2D.getX(), gyroGlobalPosition2D.getY(), gyroGlobalPosition2D.getHeading() * 180/Math.PI);
        telemetry.addData("Velocity", "X:%.1f Y:%.1f", robotCurVelocity.x, robotCurVelocity.y);
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", encHeadVel * 180/Math.PI, encHeadAccel * 180/Math.PI);
        telemetry.addLine();
    }

    public class SelfMath {
        public ElapsedTime runTime = new ElapsedTime();//Время с начала запуска программы
        public double currentTime;//В разных точках пограммы будет обозначать время когда брались значения с датчиков
        public double []oldTimes = new double[2];                                         // Предыдущее время
        public double []deltaTimes = new double[2];
        public double gyroCurHeading, gyroDeltaHeading, gyroLastHeading;
        public double deltaX , deltaY, encDeltaHeading;
        public double gyroCurHeadVel, gyroDeltaHeadVel, gyroLastHeadVel;
        private final double[] encCurVelocities = new double[3];
        private final double[] encDeltaVelocities = new double[3];
        private final double[] encLastVelocities = new double[3];
        private final double[] encCurPositions = new double[3];
        private final double[] encDeltaPositions = new double[3];
        private final double[] encLastPositions = new double[3];
        private boolean flag = false;

        public void calculateAll() {
            updateGyroHeading(AngleUnit.RADIANS);
            updateTimeGyroHead();

            updateGyroHeadVel();
            updateTimeGyroHeadVel();
            updateGyroHeadAccel();

            updatePosEncoders(false);//в тиках или сантиметрах?
            updateTimePosEnc();

            updateVelEncoders(false);
            updateTimeVelEnc();
            updateGlobalVelocity();
            updateGlobalAccel();

            updateGlobalAngle();
            updateGlobalPosition(); //затем обновляем позицию
        }

        private void updateGyroHeading(AngleUnit angleUnit) {
            gyroCurHeading = gyro.imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
        }

        private void updateTimeGyroHead() {
            gyroDeltaHeading = gyroCurHeading - gyroLastHeading;
            gyroLastHeading = gyroCurHeading;
        }
        private void updateGyroHeadVel(){
            gyroHeadVel = gyro.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

            currentTime = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков
        }
        private void updateTimeGyroHeadVel(){
            gyroDeltaHeadVel = gyroHeadVel - gyroLastHeadVel;
            gyroLastHeadVel = gyroCurHeadVel;

            deltaTimes[0] = (currentTime - oldTimes[0]) / 1000;
            oldTimes[0] = currentTime;
        }
        private void updateGyroHeadAccel(){
            if(deltaTimes[0] == 0) {gyroHeadAccel = 0; return;}

            gyroHeadAccel = gyroDeltaHeadVel / deltaTimes[0];
        }

        private void updatePosEncoders(boolean inTicks) {//Обновляем позицию с датчиков
            encCurPositions[0] = encoderClass.getCurrentPosLeft();
            encCurPositions[1] = encoderClass.getCurrentPosMid();
            encCurPositions[2] = encoderClass.getCurrentPosRight();

            if (!inTicks) {
                encCurPositions[0] = ticksToCm(encCurPositions[0]);
                encCurPositions[1] = ticksToCm(encCurPositions[1]);
                encCurPositions[2] = ticksToCm(encCurPositions[2]);
            }
        }

        private void updateTimePosEnc() {//Время привязанное к обновлению позиции
            encDeltaPositions[0] = encCurPositions[0] - encLastPositions[0];
            encLastPositions[0] = encCurPositions[0];

            encDeltaPositions[1] = encCurPositions[1] - encLastPositions[1];
            encLastPositions[1] = encCurPositions[1];

            encDeltaPositions[2] = encCurPositions[2] - encLastPositions[2];
            encLastPositions[2] = encCurPositions[2];
        }

        private void updateVelEncoders(boolean inTicks) {//Обновляем скорость с датчиков
            encCurVelocities[0] = encoderClass.getCurrentVelocityLeft();
            encCurVelocities[1] = encoderClass.getCurrentVelocityMid();
            encCurVelocities[2] = encoderClass.getCurrentVelocityRight();

            if (!inTicks) {
                encCurVelocities[0] = encCurVelocities[0] / encoderClass.COUNTS_PER_CM;
                encCurVelocities[1] = encCurVelocities[1] / encoderClass.COUNTS_PER_CM;
                encCurVelocities[2] = encCurVelocities[2] / encoderClass.COUNTS_PER_CM;
            }
            currentTime = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков
        }

        private void updateTimeVelEnc() {//Время привязанное к обновлению скорости
            encDeltaVelocities[0] = encCurVelocities[0] - encLastVelocities[0];
            encLastVelocities[0] = encCurVelocities[0];

            encDeltaVelocities[1] = encCurVelocities[1] - encLastVelocities[1];
            encLastVelocities[1] = encCurVelocities[1];

            encDeltaVelocities[2] = encCurVelocities[2] - encLastVelocities[2];
            encLastVelocities[2] = encCurVelocities[2];

            deltaTimes[1] = (currentTime - oldTimes[1]) / 1000;//время за которое программа доходит до сюда(время обновление программы)
            oldTimes[1] = currentTime;
        }

        private void updateGlobalVelocity(){
            encHeadVel = (encCurVelocities[0] - encCurVelocities[2]) / DIST_BETWEEN_ENC_X;

            robotCurVelocity.y = (encCurVelocities[0] + encCurVelocities[2]) / 2.0;
            robotCurVelocity.x = encCurVelocities[1] - encHeadVel * OFFSET_ENC_M_FROM_CENTER;
        }
        private void updateGlobalAccel() {
            if(deltaTimes[1] == 0) {encHeadAccel = 0; return;}

            double encDeltaHeadVel = (encDeltaVelocities[0] - encDeltaVelocities[2]) / DIST_BETWEEN_ENC_X;

            encHeadAccel = (encDeltaHeadVel / deltaTimes[1]);

            robotCurAccel.y = (encDeltaVelocities[0] + encDeltaVelocities[2]) / (2.0 * deltaTimes[1]);
            robotCurAccel.x = (encDeltaVelocities[1] / deltaTimes[1]) - encHeadAccel * OFFSET_ENC_M_FROM_CENTER;
        }

        private void updateGlobalAngle() {
//            encDeltaHeading = -(encDeltaPositions[0] - encDeltaPositions[2]) / DIST_BETWEEN_ENC_X;
//
//            encGlobalPosition2D.add(0, 0, encDeltaHeading * 1);
//            gyroGlobalPosition2D.add(0, 0, gyroDeltaHeading * 1);
//
////            if (encGlobalPosition2D.getHeading() < -Math.PI) {
////                encGlobalPosition2D.add(0, 0, 2 * Math.PI);
////            }
////
////            if (encGlobalPosition2D.getHeading() > Math.PI) {
////                encGlobalPosition2D.add(0, 0, -2 * Math.PI);
////            }
//
//            robotCurVelocity.rotateToGlobal(encGlobalPosition2D.getHeading());
//            robotCurAccel.rotateToGlobal(encGlobalPosition2D.getHeading());
        }

        private void updateGlobalPosition() {
            // Если перемещения не было - выходим из метода
            if (!(encDeltaPositions[0] == 0 && encDeltaPositions[1] == 0 && encDeltaPositions[2] == 0)) {
                flag = false;
            }
            if (flag) {
                return;
            }

            // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
            // Для корректной работы этот метод должен работать в непрерывном цикле

            deltaY = (encDeltaPositions[0] + encDeltaPositions[2]) / 2.0;
            deltaX = encDeltaPositions[1] - encDeltaHeading * OFFSET_ENC_M_FROM_CENTER;

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            Position2D deltaPos = new Position2D(deltaX * 1, deltaY * 1, 0);
            Position2D deltaGyroPos = new Position2D(deltaX * 1, deltaY * 1, 0);

            Vector2 rotatedVectorEnc = deltaPos.toVector().rotateToGlobal(encGlobalPosition2D.getHeading());
            Vector2 rotatedVectorGyro = deltaGyroPos.toVector().rotateToGlobal(gyroGlobalPosition2D.getHeading());

            encGlobalPosition2D.add(rotatedVectorEnc.x * 1, rotatedVectorEnc.y * 1, 0);
            gyroGlobalPosition2D.add(rotatedVectorGyro.x * 1, rotatedVectorGyro.y * 1, 0);

            if (encDeltaPositions[0] == 0 && encDeltaPositions[1] == 0 && encDeltaPositions[2] == 0) {
                flag = true;
            }
        }
    }
}