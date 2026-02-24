package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public class DrivetrainMotors extends MotorModule {
    public EncoderClass encoderClass;
    public GyroscopeClass gyro;
    public DrivetrainMotors(OpMode op, GyroscopeClass gyro) {
        super(op);

        motorsWrapper
                .add(op, dcMotorEx.initialize(op,"rightB").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT))
                .add(op, dcMotorEx.initialize(op,"rightF").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT))
                .add(op, dcMotorEx.initialize(op,"leftF").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.REVERSE).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT))
                .add(op, dcMotorEx.initialize(op,"leftB").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.REVERSE).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT));

        encoderClass = new EncoderClass(op, isInitialized);
        this.gyro = gyro;

        sayInited();
    }
    public void setPower(double forwardPow, double sidePow, double anglePow){
        if (!isInitialized) return;

        double minPow = 0.13 / voltageSensorClass.getkPower();

        if(voltageSensorClass.getkPower() != 0){
            forwardPow = forwardPow * 12.5 / voltageSensorClass.getCurVoltage();
            sidePow = sidePow * 12.5 / voltageSensorClass.getCurVoltage();
            anglePow = anglePow * 12.5 / voltageSensorClass.getCurVoltage();
        }

//        if(forwardPow != 0 && Math.abs(forwardPow) < minPow) forwardPow = Math.signum(forwardPow) * minPow;
//        if(sidePow != 0 &&  Math.abs(sidePow) < minPow) sidePow = Math.signum(sidePow) * minPow;
//        if(anglePow != 0 &&  Math.abs(anglePow) < minPow) anglePow = Math.signum(anglePow) * minPow;

        double powRightB = forwardPow - sidePow + anglePow;
        double powRightF = forwardPow + sidePow + anglePow;
        double powlLeftF = forwardPow - sidePow - anglePow;
        double powLeftB = forwardPow + sidePow - anglePow;

        motorsWrapper.get("rightB").setPower(powRightB);
        motorsWrapper.get("rightF").setPower(powRightF);
        motorsWrapper.get("leftF").setPower(powlLeftF);
        motorsWrapper.get("leftB").setPower(powLeftB);

        encoderClass.update();
        gyro.update();
    }


    @Override
    public void showData() {
        motorsWrapper.showData();
//        telemetry.addLine("===MOTORS===");
//        telemetry.addData("Powers right side", "RF:%.2f RB:%.2f",  motorsWrapper.get("rightF").getPower(), motorsWrapper.get("rightB").getPower());
//        telemetry.addData("Powers left side", "LF:%.2f LB:%.2f", motorsWrapper.get("leftF").getPower(),  motorsWrapper.get("leftB").getPower());
//        telemetry.addLine();
//        encoderClass.showData();
    }

    public class EncoderClass extends UpdatableModule {
        public EncoderClass(OpMode op, boolean isInit){
            super(op);

            selfMath = new SelfMath();
            encodersBuffer = new OdometryBuffer();
            this.isInitialized = isInit;
            sayInited();
        }
        public OdometryBuffer encodersBuffer;
        private SelfMath selfMath;
        private double COUNTS_PER_ENCODER_REV = 2000;
        private double DRIVE_GEAR_REDUCTION = 1;
        private double ENC_WHEEL_DIAM_CM = 4.8;
        private double COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
                (ENC_WHEEL_DIAM_CM * Math.PI);

        private boolean switcher = false;//Переключение системы исчисления

        private double ticksToCm(double ticks){
            return ticks / COUNTS_PER_CM;
        }
        public double getCurentPos(String motorName){
            double pos = -motorsWrapper.get(motorName).getMotor().getCurrentPosition();
            return switcher ? pos : ticksToCm(pos);
        }
        public double getCurrentVelocity(String motorName){
            double vel = -motorsWrapper.get(motorName).getMotorEx().getVelocity();
            return  switcher ? vel : ticksToCm(vel);
        }

        @Override
        public void update() {
            selfMath.calculateAll();
        }

        @Override
        public void showData() {
            telemetry.addLine("===ENCODERS===");
            telemetry.addData("Left pos", getCurentPos("left"));
            telemetry.addData("Mid pos", getCurentPos("Mid"));
            telemetry.addData("Right pos", getCurentPos("Right"));
            telemetry.addLine();
        }
        public class SelfMath {
            private OdometryData rawData;
            private final double[] encCurVelocities, encDeltaVelocities, encLastVelocities;
            private final double[] encCurPositions, encDeltaPositions, encLastPositions;
            public ElapsedTime runTime ;//Время с начала запуска программы
            public double []currentTime, deltaTimes, oldTimes;//В разных точках пограммы будет обозначать время когда брались значения с датчиков
            private boolean flag = false;
            private double fltrdVelLeft, fltrdVelMid, fltrdVelRight;
            public SelfMath(){
                currentTime = new double[2];
                oldTimes = new double[2];                                         // Предыдущее время
                deltaTimes = new double[2];

                encCurVelocities = new double[3];
                encDeltaVelocities = new double[3];
                encLastVelocities = new double[3];
                encCurPositions = new double[3];
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
                encCurPositions[0] = getCurentPos("encLeft");
                encCurPositions[1] = getCurentPos("encMid");
                encCurPositions[2] = getCurentPos("encRight");

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
                fltrdVelLeft = filtr * getCurrentVelocity("encLeft") + (1 - filtr) * fltrdVelLeft;
                fltrdVelMid = filtr * getCurrentVelocity("encMid") + (1 - filtr) * fltrdVelMid;
                fltrdVelRight = filtr * getCurrentVelocity("encRight") + (1 - filtr) * fltrdVelRight;

                encCurVelocities[0] = fltrdVelLeft;
                encCurVelocities[1] = fltrdVelMid;
                encCurVelocities[2] = fltrdVelRight;

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
