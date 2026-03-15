package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.Units;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.ExecutableCollector;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class DrivetrainMotors extends ExecutableCollector {
    public EncoderClass encoderClass;
    public DrivetrainMotors() {
        super(false);
        createMotorWrapperUtils();
        motorsCollector
                .add(motorBuilder.initialize(rightBack).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(13.0, 1.026, 2000.0, 4.8, 1.0).get())
                .add(motorBuilder.initialize(rightFront).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(13.0, 0.976, 2000.0, 4.8, 1.0).get())
                .add(motorBuilder.initialize(leftFront).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.REVERSE).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(13.0, 1.053, 2000.0, 4.8, 1.0).get())
                .add(motorBuilder.initialize(leftBack).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.REVERSE).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(13.0, 1.053, 0.0, 5.0, 1.0).get());

        encoderClass = new EncoderClass(motorsCollector);
        sayCreated();
    }

    public String rightBack = expansionHubDevices.getMotor(0);
    public String rightFront = expansionHubDevices.getMotor(1);
    public String leftFront = expansionHubDevices.getMotor(2);
    public String leftBack = expansionHubDevices.getMotor(3);

    public String encRight = expansionHubDevices.getEncoder(0);
    public String encLeft = expansionHubDevices.getEncoder(2);
    public String encMid = expansionHubDevices.getEncoder(1);
    public String testEnc = expansionHubDevices.getEncoder(3);

    @Override
    protected void executeExt(@NonNull Double... args) {
        double forwardPow = args[0];
        double sidePow = args[1];
        double anglePow = args[2];

        motorsCollector.get(rightBack).execute(forwardPow - sidePow + anglePow);
        motorsCollector.get(rightFront).execute(forwardPow + sidePow + anglePow);
        motorsCollector.get(leftFront).execute(forwardPow - sidePow - anglePow);
        motorsCollector.get(leftBack).execute(forwardPow + sidePow - anglePow);
    }
    @Override
    public void showDataExt() {
        motorsCollector.showData();
        encoderClass.showData();
    }
    public class EncoderClass extends UpdatableCollector {
        public OdometryBuffer encodersBuffer;
        private OdometryData rawData;
        private final double[] encCurVelocities, encDeltaVelocities, encLastVelocities;
        private final double[] encCurPositions, encDeltaPositions, encLastPositions;
        public ElapsedTime runTime ;//Время с начала запуска программы
        public double []currentTime, deltaTimes, oldTimes;//В разных точках пограммы будет обозначать время когда брались значения с датчиков
        private double fltrdVelLeft, fltrdVelMid, fltrdVelRight;
        public EncoderClass(MotorWrapper.InnerCollector motors) {
            super(false);
            motorsCollector = motors;
            encodersBuffer = new OdometryBuffer();

            currentTime = new double[2];
            oldTimes = new double[2];                                         // Предыдущее время
            deltaTimes = new double[2];

            encCurVelocities = new double[4];
            encDeltaVelocities = new double[3];
            encLastVelocities = new double[3];
            encCurPositions = new double[4];
            encDeltaPositions = new double[3];
            encLastPositions = new double[3];

            rawData = new OdometryData();
            runTime = new ElapsedTime();
            sayCreated();
        }
        @Override
        protected void updateExt() {
            updatePos();
            updateVel();

            updateDeltasPos();
            updateDeltasVel();

            updateVels();
            updateAccels();

            updateGlobalPosition();

            encodersBuffer.beginWrite().set(rawData);
            encodersBuffer.endWrite2();
        }
        private void updatePos() {//Обновляем позицию с датчиков
            encCurPositions[0] = motorsCollector.get(encLeft).getCurPos(Units.Cm);
            encCurPositions[1] = motorsCollector.get(encMid).getCurPos(Units.Cm);
            encCurPositions[2] = motorsCollector.get(encRight).getCurPos(Units.Cm);
            encCurPositions[3] = motorsCollector.get(testEnc).getCurPos(Units.Cm);

            currentTime[0] = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков
        }

        private void updateDeltasPos() {//Время привязанное к обновлению позиции
            encDeltaPositions[0] = encCurPositions[0] - encLastPositions[0];
            encLastPositions[0] = encCurPositions[0];

            encDeltaPositions[1] = encCurPositions[1] - encLastPositions[1];
            encLastPositions[1] = encCurPositions[1];

            encDeltaPositions[2] = encCurPositions[2] - encLastPositions[2];
            encLastPositions[2] = encCurPositions[2];

            deltaTimes[0] = (currentTime[0] - oldTimes[1]) / 1000;//время за которое программа доходит до сюда(время обновление программы)
            oldTimes[0] = currentTime[0];
        }

        private void updateVel() {//Обновляем скорость с датчиков
            double filtr = 0.3;
            fltrdVelLeft = filtr * motorsCollector.get(encLeft).getCurVel(Units.Cm)  + (1 - filtr) * fltrdVelLeft;
            fltrdVelMid = filtr * motorsCollector.get(encMid).getCurVel(Units.Cm) + (1 - filtr) * fltrdVelMid;
            fltrdVelRight = filtr * motorsCollector.get(encRight).getCurVel(Units.Cm) + (1 - filtr) * fltrdVelRight;

            encCurVelocities[0] = fltrdVelLeft;
            encCurVelocities[1] = fltrdVelMid;
            encCurVelocities[2] = fltrdVelRight;
            encCurVelocities[3] = motorsCollector.get(testEnc).getCurVel(Units.Cm);

            currentTime[1] = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков
        }

        private void updateDeltasVel() {//Время привязанное к обновлению скорости
            encDeltaVelocities[0] = encCurVelocities[0] - encLastVelocities[0];
            encLastVelocities[0] = encCurVelocities[0];

            encDeltaVelocities[1] = encCurVelocities[1] - encLastVelocities[1];
            encLastVelocities[1] = encCurVelocities[1];

            encDeltaVelocities[2] = encCurVelocities[2] - encLastVelocities[2];
            encLastVelocities[2] = encCurVelocities[2];

            deltaTimes[1] = (currentTime[1] - oldTimes[1]) / 1000;//время за которое программа доходит до сюда(время обновление программы)
            oldTimes[1] = currentTime[1];
        }

        private void updateVels(){
            double robotHeadVel = (encCurVelocities[0] - encCurVelocities[2]) / DIST_BETWEEN_ENC_X;

            Vector2 robotCurVelocity = new Vector2();

            robotCurVelocity.y = (encCurVelocities[0] + encCurVelocities[2]) / 2.0;
            robotCurVelocity.x = encCurVelocities[1] - robotHeadVel * OFFSET_ENC_M_FROM_CENTER;

            rawData.setHeadVel(robotHeadVel);
            rawData.setVelocity(robotCurVelocity);
        }
        private void updateAccels() {
            double robotHeadAccel;

            double encDeltaHeadVel = (encDeltaVelocities[0] - encDeltaVelocities[2]) / DIST_BETWEEN_ENC_X;

            if(deltaTimes[1] == 0) {robotHeadAccel = 0; }
            else robotHeadAccel = (encDeltaHeadVel / deltaTimes[1]);

            Vector2 robotAccel = new Vector2();

            robotAccel.y = (encDeltaVelocities[0] + encDeltaVelocities[2]) / (2.0 * deltaTimes[1]);
            robotAccel.x = (encDeltaVelocities[1] / deltaTimes[1]) - robotHeadAccel * OFFSET_ENC_M_FROM_CENTER;

            rawData.setHeadAccel(robotHeadAccel);
            rawData.setAccel(robotAccel);
        }

        private void updateGlobalPosition() {
            // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
            // Для корректной работы этот метод должен работать в непрерывном цикле
            double encDeltaHeading = -(encDeltaPositions[0] - encDeltaPositions[2]) / DIST_BETWEEN_ENC_X;
            double deltaY = (encDeltaPositions[0] + encDeltaPositions[2]) / 2.0;
            double deltaX = encDeltaPositions[1] - encDeltaHeading * OFFSET_ENC_M_FROM_CENTER;

            Vector2 rotatedVector2 = new Vector2(deltaX, deltaY).rotateToGlobal(encDeltaHeading);
            rawData.setPosition(new Position2D(rotatedVector2.x, rotatedVector2.y, 0));
        }
        @Override
        protected void showDataExt() {
            telemetry.addLine("Left");
            telemetry.addData("Data", "Pos %.2f Vel %.2f",encCurPositions[0], encCurVelocities[0]);
            telemetry.addLine("Mid");
            telemetry.addData("Data", "Pos %.2f Vel %.2f",encCurPositions[1], encCurVelocities[1]);
            telemetry.addLine("Right");
            telemetry.addData("Data", "Pos %.2f Vel %.2f",encCurPositions[2], encCurVelocities[2]);
            telemetry.addLine("Test");
            telemetry.addData("Data", "Pos %.2f Vel %.2f",encCurPositions[3], encCurVelocities[3]);
        }
    }

}
