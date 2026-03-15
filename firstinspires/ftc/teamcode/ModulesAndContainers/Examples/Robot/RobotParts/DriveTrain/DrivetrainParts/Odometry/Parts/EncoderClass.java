package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;

public class EncoderClass extends UpdatableModule {
    private DcMotorEx encLeft;
    private DcMotorEx encMid;
    private DcMotorEx encRight;
    public EncoderClass(OpMode op){
        super(op);

        try {
            encLeft = hardwareMap.get(DcMotorEx.class, "leftF");
            encMid = hardwareMap.get(DcMotorEx.class, "rightB" );
            encRight = hardwareMap.get(DcMotorEx.class, "rightF");
        } catch (Exception e) {
            isInizialized = false;
            return;
        }

        encRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем правый энкодер
        encLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем левый энкодер
        encMid.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем средний энкодер

        encLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encMid.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rawData = new OdometryData();
        selfMath = new SelfMath();
        telemetry.addLine("Encoders Inited");
    }
    private OdometryData rawData;
    private SelfMath selfMath;
    private double COUNTS_PER_ENCODER_REV = 2000;
    private double DRIVE_GEAR_REDUCTION = 1;
    private double ENC_WHEEL_DIAM_CM = 4.8;
    private double COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
            (ENC_WHEEL_DIAM_CM * Math.PI);

    private boolean switcher = false;//Переключение системы исчисления

    public OdometryData getRawData() {
        return rawData;
    }

    private double ticksToCm(double ticks){
        return ticks / COUNTS_PER_CM;
    }
    public double getCurrentPosLeft(){
        return switcher ? -encLeft.getCurrentPosition() : ticksToCm(-encLeft.getCurrentPosition());
    }

    public double getCurrentPosMid(){
        return switcher ? -encMid.getCurrentPosition() : ticksToCm(-encMid.getCurrentPosition());
    }

    public double getCurrentPosRight(){
        return switcher ? -encRight.getCurrentPosition() : ticksToCm(-encRight.getCurrentPosition());
    }

    public double getCurrentVelocityLeft(){
        return  switcher ? -encLeft.getVelocity() : ticksToCm(-encLeft.getVelocity());
    }

    public double getCurrentVelocityMid(){
        return switcher ? -encMid.getVelocity() : ticksToCm(-encMid.getVelocity());
    }

    public double getCurrentVelocityRight(){
        return switcher ? -encRight.getVelocity() : ticksToCm(-encLeft.getVelocity());
    }

    @Override
    public void update() {
        if (!isInizialized) return;
        selfMath.calculateAll();
    }

    @Override
    public void showData() {
        telemetry.addLine("===ENCODERS===");

        if(isInizialized){
            telemetry.addData("Left", getCurrentPosLeft());
            telemetry.addData("Right", getCurrentPosRight());
            telemetry.addData("Mid", getCurrentPosMid());
        }else {
            telemetry.addLine("DEVICE NOT FOUND");
        }

        telemetry.addLine();
    }
   private class SelfMath {
        private final double[] encCurVelocities;
        private final double[] encDeltaVelocities;
        private final double[] encLastVelocities;
        private final double[] encCurPositions;
        private final double[] encDeltaPositions;
        private final double[] encLastPositions;
        public ElapsedTime runTime ;//Время с начала запуска программы
        public double []currentTime;//В разных точках пограммы будет обозначать время когда брались значения с датчиков
        public final double []deltaTimes;
        public final double [] oldTimes;// Предыдущее время
        private boolean flag = false;

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
        }


        private void updatePosEncoders() {//Обновляем позицию с датчиков
            encCurPositions[0] = getCurrentPosLeft();
            encCurPositions[1] = getCurrentPosMid();
            encCurPositions[2] = getCurrentPosRight();

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
            encCurVelocities[0] = getCurrentVelocityLeft();
            encCurVelocities[1] = getCurrentVelocityMid();
            encCurVelocities[2] = getCurrentVelocityRight();

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

            rawData.setRobotHeadVel(robotHeadVel);
            rawData.setRobotVelocity(robotCurVelocity);
        }
        private void updateAccelsEncoders() {
            double robotHeadAccel;

            double encDeltaHeadVel = (encDeltaVelocities[0] - encDeltaVelocities[2]) / DIST_BETWEEN_ENC_X;

            if(deltaTimes[1] == 0) {robotHeadAccel = 0; }
            else robotHeadAccel = (encDeltaHeadVel / deltaTimes[1]);

            Vector2 robotAccel = new Vector2();

            robotAccel.y = (encDeltaVelocities[0] + encDeltaVelocities[2]) / (2.0 * deltaTimes[1]);
            robotAccel.x = (encDeltaVelocities[1] / deltaTimes[1]) - robotHeadAccel * OFFSET_ENC_M_FROM_CENTER;

            rawData.setRobotHeadAccel(robotHeadAccel);
            rawData.setRobotAccel(robotAccel);
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
            double encDeltaHeading = -(encDeltaPositions[0] - encDeltaPositions[2]) / DIST_BETWEEN_ENC_X;
            double deltaY = (encDeltaPositions[0] + encDeltaPositions[2]) / 2.0;
            double deltaX = encDeltaPositions[1] - encDeltaHeading * OFFSET_ENC_M_FROM_CENTER;

            rawData.setRobotPosition(new Position2D(deltaX, deltaY, encDeltaHeading));

            if (encDeltaPositions[0] == 0 && encDeltaPositions[1] == 0 && encDeltaPositions[2] == 0) {
                flag = true;
            }
        }
    }
}
