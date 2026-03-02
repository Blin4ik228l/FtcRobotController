package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.Units;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.EncoderClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.InnerMath;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree.DeviceManager;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutingModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.ExecutableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

public class DrivetrainMotors extends ExecutingModule {
    public EncoderClass2 encoderClass;
    public DrivetrainMotors(MainFile mainFile) {
        super(mainFile);

        createMotorWrapperUtils();
        motorsCollector
                .add(motorBuilder.initialize(mainFile, rightBack).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields( 12.5, 1.0).get())
                .add(motorBuilder.initialize(mainFile, rightFront).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields( 12.5, 1.0).get())
                .add(motorBuilder.initialize(mainFile, leftFront).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.REVERSE).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields( 12.5, 1.0).get())
                .add(motorBuilder.initialize(mainFile, leftBack).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.REVERSE).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                        .setFields(12.5, 1.0).get());

        encoderClass = new EncoderClass2(mainFile, motorsCollector);
        sayCreated();
    }

    public String rightBack = expansionHubDevices.getMotor(0);
    public String rightFront = expansionHubDevices.getMotor(1);
    public String leftFront = expansionHubDevices.getMotor(2);
    public String leftBack = expansionHubDevices.getMotor(3);

    public String encRight = expansionHubDevices.getEncoder(0);
    public String encLeft = expansionHubDevices.getEncoder(1);
    public String encMid = expansionHubDevices.getEncoder(2);
    public String testEnc = expansionHubDevices.getEncoder(3);

    @Override
    public void showDataExt() {
        motorsCollector.showData();
        encoderClass.showData();
    }
    @Override
    protected void executeExt(Double... args) {
        double forwardPow = args[0];
        double sidePow = args[1];
        double anglePow = args[2];

        motorsCollector.get(rightBack).execute(forwardPow - sidePow + anglePow);
        motorsCollector.get(rightFront).execute(forwardPow + sidePow + anglePow);
        motorsCollector.get(leftFront).execute(forwardPow - sidePow - anglePow);
        motorsCollector.get(leftBack).execute(forwardPow + sidePow - anglePow);

        encoderClass.update();
    }
    public class EncoderClass2 extends UpdatingModule {
        private double COUNTS_PER_ENCODER_REV = 2000;
        private double DRIVE_GEAR_REDUCTION = 1;
        private double ENC_WHEEL_DIAM_CM = 4.8;
        public OdometryBuffer encodersBuffer;
        private SelfMath selfMath;

        public EncoderClass2(MainFile mainFile, MotorWrapper.InnerCollector motors) {
            super(mainFile);
            motorsCollector = motors;
            selfMath = new SelfMath();
            encodersBuffer = new OdometryBuffer();
            sayCreated();
        }
        public InnerMath test = new InnerMath().setCOUNTS_PER_ENCODER_REV(384.5).setRadius(5).setDRIVE_GEAR_REDUCTION(1).calculateCountsPerCm();
        @Override
        protected void updateExt() {
            selfMath.calculateAll();
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("Left pos", selfMath.encCurPositions[0]);
            telemetry.addData("Left speed", selfMath.encCurVelocities[0]);
            telemetry.addData("Mid speed", selfMath.encCurVelocities[1]);
            telemetry.addData("Mid pos", selfMath.encCurPositions[1]);
            telemetry.addData("Right speed", selfMath.encCurVelocities[2]);
            telemetry.addData("Right pos", selfMath.encCurPositions[2]);



            telemetry.addData("Test pos", test.getCurentPos(motorsCollector.get(testEnc), Units.Cm));
            telemetry.addData("Test speed", test.getCurrentVelocity(motorsCollector.get(testEnc), Units.Cm));
        }
        public class SelfMath extends InnerMath {
            private OdometryData rawData;
            private final double[] encCurVelocities, encDeltaVelocities, encLastVelocities;
            private final double[] encCurPositions, encDeltaPositions, encLastPositions;
            public ElapsedTime runTime ;//Время с начала запуска программы
            public double []currentTime, deltaTimes, oldTimes;//В разных точках пограммы будет обозначать время когда брались значения с датчиков
            private double fltrdVelLeft, fltrdVelMid, fltrdVelRight;
            public SelfMath(){
                setRadius(ENC_WHEEL_DIAM_CM / 2.0)
                        .setCOUNTS_PER_ENCODER_REV(COUNTS_PER_ENCODER_REV)
                        .setDRIVE_GEAR_REDUCTION(DRIVE_GEAR_REDUCTION)
                        .calculateCountsPerCm();

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
            }
            public void calculateAll(){
                updatePosEncoders();
                updateVelEncoders();

                updateDeltasPosEnc();
                updateDeltasVelEnc();

                updateVelsEncoders();
                updateAccelsEncoders();

                updateGlobalPosition();

                encodersBuffer.beginWrite().set(rawData);
                encodersBuffer.endWrite2();
            }


            private void updatePosEncoders() {//Обновляем позицию с датчиков
                encCurPositions[0] = getCurentPos(motorsCollector.get(encLeft), Units.Cm);
                encCurPositions[1] = getCurentPos(motorsCollector.get(encMid), Units.Cm);
                encCurPositions[2] = getCurentPos(motorsCollector.get(encRight), Units.Cm);
                encCurPositions[3] = getCurentPos(motorsCollector.get(testEnc), Units.Cm);

                currentTime[0] = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков
            }

            private void updateDeltasPosEnc() {//Время привязанное к обновлению позиции
                encDeltaPositions[0] = encCurPositions[0] - encLastPositions[0];
                encLastPositions[0] = encCurPositions[0];

                encDeltaPositions[1] = encCurPositions[1] - encLastPositions[1];
                encLastPositions[1] = encCurPositions[1];

                encDeltaPositions[2] = encCurPositions[2] - encLastPositions[2];
                encLastPositions[2] = encCurPositions[2];

                deltaTimes[0] = (currentTime[0] - oldTimes[1]) / 1000;//время за которое программа доходит до сюда(время обновление программы)
                oldTimes[0] = currentTime[0];
            }

            private void updateVelEncoders() {//Обновляем скорость с датчиков
                double filtr = 0.3;
                fltrdVelLeft = filtr * getCurrentVelocity(motorsCollector.get(encLeft), Units.Cm) + (1 - filtr) * fltrdVelLeft;
                fltrdVelMid = filtr * getCurrentVelocity(motorsCollector.get(encMid), Units.Cm) + (1 - filtr) * fltrdVelMid;
                fltrdVelRight = filtr * getCurrentVelocity(motorsCollector.get(encRight), Units.Cm) + (1 - filtr) * fltrdVelRight;

                encCurVelocities[0] = fltrdVelLeft;
                encCurVelocities[1] = fltrdVelMid;
                encCurVelocities[2] = fltrdVelRight;
                encCurVelocities[3] = getCurentPos(motorsCollector.get(testEnc), Units.Cm);

                currentTime[1] = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков
            }

            private void updateDeltasVelEnc() {//Время привязанное к обновлению скорости
                encDeltaVelocities[0] = encCurVelocities[0] - encLastVelocities[0];
                encLastVelocities[0] = encCurVelocities[0];

                encDeltaVelocities[1] = encCurVelocities[1] - encLastVelocities[1];
                encLastVelocities[1] = encCurVelocities[1];

                encDeltaVelocities[2] = encCurVelocities[2] - encLastVelocities[2];
                encLastVelocities[2] = encCurVelocities[2];

                deltaTimes[1] = (currentTime[1] - oldTimes[1]) / 1000;//время за которое программа доходит до сюда(время обновление программы)
                oldTimes[1] = currentTime[1];
            }

            private void updateVelsEncoders(){
                double robotHeadVel = (encCurVelocities[0] - encCurVelocities[2]) / DIST_BETWEEN_ENC_X;

                Vector2 robotCurVelocity = new Vector2();

                robotCurVelocity.y = (encCurVelocities[0] + encCurVelocities[2]) / 2.0;
                robotCurVelocity.x = encCurVelocities[1] - robotHeadVel * OFFSET_ENC_M_FROM_CENTER;

                rawData.setHeadVel(robotHeadVel);
                rawData.setVelocity(robotCurVelocity);
            }
            private void updateAccelsEncoders() {
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

                rawData.setPosition(new Position2D(deltaX, deltaY, encDeltaHeading));
            }
        }
    }

}
