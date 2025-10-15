package org.firstinspires.ftc.teamcode.Robot.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.EncoderClass;
import org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.MathUtils.Position;
import org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.TeamColor;

public class ExOdometry extends Module {
    //Все энкодеры на телеге + гироскоп + камера  составляющие общую систему оценки положения робота в пространстве.
    public ExOdometry(OpMode op, TeamColor teamColor){
        super(op.telemetry);
//            voltageSensor = op.hardwareMap.get(VoltageSensor.class, "");

        gyro = new GyroscopeClass(op);
        encoderClass = new EncoderClass(op);
        selfMath = new SelfMath();
        selfMath.deltaTimes = new double[3];
        selfMath.oldTimes = new double[3];

        encGlobalPosition = new Position();
        gyroGlobalPosition = new Position();

        camera = new CameraClass(op, teamColor);//Ждём пока камера не начнёт стримить
    }
    public GyroscopeClass gyro;
    public CameraClass camera;
    public EncoderClass encoderClass;
    public final SelfMath selfMath;
    public final Position gyroGlobalPosition;                                   // Относительное перемещение
    public final Position encGlobalPosition;
    public VoltageSensor voltageSensor;
    public Vector2[] maxVels = new Vector2[3];
    public Vector2[] maxAccels = new Vector2[3];
    public Vector2[] encodersVelNotCentric = new Vector2[3];
    public Vector2[] encodersVelCentric = new Vector2[3];
    public Vector2[] encodersAccelNotCentric = new Vector2[3];
    public Vector2[] encodersAccelCentric = new Vector2[3];
    public Vector2 robotSelfCentricVel = new Vector2();
    public Vector2 robotSelfNotCentricVel = new Vector2();
    public Vector2 robotSelfCentricAccel = new Vector2();
    public Vector2 robotSelfNotCentricAccel = new Vector2();
    public Vector2 robotSelfAngleVelCentric = new Vector2();
    public Vector2 robotSelfAngleAccelCentric = new Vector2();

    private double ticksToCm(double ticks){
        return ticks / encoderClass.COUNTS_PER_CM;
    }

    public void updateAll(){
        selfMath.calculateAll();
    }

    public void showEncodersVel(){
        telemetry.addLine("Encoders Vel");
        telemetry.addLine("\nLeftEncoder")
                .addData("\nVel", encodersVelCentric[0].length())
                .addData("\nVelX", encodersVelCentric[0].x)
                .addData("\nVelY", encodersVelCentric[0].y);
        telemetry.addLine("\nMidEncoder")
                .addData("\nVel", encodersVelCentric[1].length())
                .addData("\nVelX", encodersVelCentric[1].x)
                .addData("\nVelY", encodersVelCentric[1].y);
        telemetry.addLine("\nRightEncoder")
                .addData("\nVel", encodersVelCentric[2].length())
                .addData("\nVelX", encodersVelCentric[2].x)
                .addData("VelY", encodersVelCentric[2].y);
        telemetry.addLine();
    }

    public void showRobotVel(){
        telemetry.addLine("Robot Vel")
                .addData("\nVel", robotSelfCentricVel.length())
                .addData("\nVelX", robotSelfCentricVel.x)
                .addData("VelY", robotSelfCentricVel.y);
        telemetry.addLine();
    }

    public void showEncodersAccel(){
        telemetry.addLine("Encoders Accel");
        telemetry.addLine("\nLeftEncoder")
                .addData("\nAccel", encodersAccelCentric[0].length())
                .addData("\nAccelX", encodersAccelCentric[0].x)
                .addData("\nAccelY", encodersAccelCentric[0].y);
        telemetry.addLine("\nMidEncoder")
                .addData("\nAccel", encodersAccelCentric[1].length())
                .addData("\nAccelX", encodersAccelCentric[1].x)
                .addData("\nAccelY", encodersAccelCentric[1].y);
        telemetry.addLine("\nRightEncoder")
                .addData("\nAccel", encodersAccelCentric[2].length())
                .addData("\nAccelX", encodersAccelCentric[2].x)
                .addData("AccelY", encodersAccelCentric[2].y);
        telemetry.addLine();
    }

    public void showRobotAccel(){
        telemetry.addLine("Robot Accel")
                .addData("\nAccel", robotSelfCentricAccel.length())
                .addData("\nAccelX", robotSelfCentricAccel.x)
                .addData("AccelY", robotSelfCentricAccel.y);
        telemetry.addLine();
    }

    public void showEncPositions(){
        telemetry.addLine("Encoders position")
                .addData("\nLeft", selfMath.encCurPositions[0])
                .addData("\nMid",  selfMath.encCurPositions[1])
                .addData("Right",  selfMath.encCurPositions[2]);
        telemetry.addLine();
    }
    public void showRobotPositionEnc(){
        telemetry.addLine("Robot pos with encAngle")
                .addData("\nX", encGlobalPosition.getX())
                .addData("\nY",  encGlobalPosition.getY())
                .addData("Heading", encGlobalPosition.getHeading() * 180/Math.PI);
        telemetry.addLine();
    }

    public void showRobotPositionGyro(){
        telemetry.addLine("Robot pos with gyroAngle")
                .addData("\nX", gyroGlobalPosition.getX())
                .addData("\nY",  gyroGlobalPosition.getY())
                .addData("Heading", gyroGlobalPosition.getHeading() * 180/Math.PI);
        telemetry.addLine();
    }




//        public Vector2 getMaxVelLeft() {
//            if(maxVelLeft.length() < getEncLeftVelocity(false).length()) maxVelLeft = getEncLeftVelocity(false);
//            return maxVelLeft;
//        }
//
//        public Vector2 getMaxVelRight() {
//            if(maxVelRight.length() < getEncRightVelocity(false).length()) maxVelRight = getEncRightVelocity(false);
//            return maxVelRight;
//        }
//
//        public Vector2 getMaxVelMid() {
//            if(maxVelMid.length() < getEncMidVelocity(false).length()) maxVelMid = getEncMidVelocity(false);
//            return maxVelMid;
//        }
//
//        public Vector2 getMaxAccelLeft() {
//            if(maxAccelLeft.length() < getEncLeftAccel(false).length()) maxAccelLeft = getEncLeftAccel(false);
//            return maxAccelLeft;
//        }
//
//        public Vector2 getMaxAccelRight() {
//            if(maxAccelRight.length() < getEncRightAccel(false).length()) maxAccelRight = getEncRightAccel(false);
//            return maxAccelRight;
//        }
//
//        public Vector2 getMaxAccelMid() {
//            if(maxAccelMid.length() < getEncMidAccel(false).length()) maxAccelMid = getEncMidAccel(false);
//            return maxAccelMid;
//        }



//        public void showEncodersVelocityComponents(){
//            telemetry.addLine("Encoders Velo:")
//                    .addData("\nLeft encoder X", getEncLeftVelocity(false).x)
//                    .addData("\nLeft encoder Y", getEncLeftVelocity(false).y)
//                    .addData("\nMid encoder X", getEncMidVelocity(false).x)
//                    .addData("\nMid encoder X", getEncMidVelocity(false).y)
//                    .addData("\nRight encoder X", getEncRightVelocity(false).x)
//                    .addData("\nRight encoder Y", getEncRightVelocity(false).y);
//            telemetry.addLine();
//        }

    public class SelfMath{
        public ElapsedTime runTime = new ElapsedTime();//Время с начала запуска программы
        public double currentTime = 0;//В разных точках пограммы будет обозначать время когда брались значения с энкодеров
        public double oldTimes[] = new double[3];                                         // Предыдущее время
        public double deltaTimes[] = new double[3];
        public double yaw = 0;
        public double deltaYaw = 0;
        public double lastYaw = 0;
        public double angleVel = 0;
        public double angleAccel = 0;

        public double g_X = 0;
        public double g_Y = 0;
        public double g_Heading = 0;
        public double deltaHeadingEnc = 0;
        public double deltaX = 0;
        public double deltaY = 0;
        public double deltaHeadingGyro = 0;
        private final double[] encCurVelocities = new double[3];
        private double leftVelCur = 0;
        private double midVelCur = 0;
        private double rightVelCur = 0;
        private final double[] encDeltaVelocities = new double[3];
        private final double[] encLastVelocities = new double[3];
        private final double[] encCurPositions = new double[3];
        private final double[] encDeltaPositions = new double[3];
        private final double[] encLastPositions = new double[3];
        public void calculateAll(){
            updateAngle(AngleUnit.RADIANS);
            updateTimeToUpdAngl();

            updatePosFromEncoders(false);//в тиках или сантиметрах?
            updateTimeToUpdPosFrmEnc();

            updateVelFromEncoders(false);
            updateTimeToUpdVelFrmEnc();

            updateEncodersVelocityNotCentric();//Обновляем скорость
            updateEncodersVelocityCentric();

            updateRobotVelocityNotCentric();//Обновляем ускорение
            updateRobotVelocityCentric();

            updateEncodersAccelNotCentric();
            updateEncodersAccelCentric();

            updateRobotAccelNotCentric();
            updateRobotAccelCentric();

            updateGlobalAngle();
            updateGlobalPosition(); //затем обновляем позицию
        }

        private void updateAngle(AngleUnit angleUnit){
            yaw = gyro.imu.getRobotYawPitchRollAngles().getYaw(angleUnit);

            currentTime = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков
        }

        private void updateTimeToUpdAngl(){
            deltaYaw = yaw - lastYaw;
            lastYaw = yaw;

            deltaTimes[2] = (currentTime - oldTimes[2])/1000;//время за которое программа доходит до сюда(время обновление программы)
            oldTimes[2] = currentTime;
        }

        private void updateGlobalAngle(){
            deltaHeadingGyro = deltaYaw;

            if(encGlobalPosition.getHeading() < -Math.PI) {
                encGlobalPosition.add(0,0, 2 * Math.PI);
                }

            if(encGlobalPosition.getHeading() > Math.PI) {
                encGlobalPosition.add(0,0, -2 * Math.PI);
                }
            deltaHeadingEnc = -(encDeltaPositions[0] - encDeltaPositions[2]) / DIST_BETWEEN_ENC_X;


            encGlobalPosition.add(0, 0, deltaHeadingEnc * 1);
            gyroGlobalPosition.add(0, 0, deltaHeadingGyro * 1);
        }

        private void updateGlobalPosition(){
            // Если перемещения не было - выходим из метода
            if(encDeltaPositions[0] == 0 && encDeltaPositions[1] == 0 && encDeltaPositions[2] == 0 ) {
                return;
            }

            // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
            // Для корректной работы этот метод должен работать в непрерывном цикле

            deltaY = (encDeltaPositions[0] + encDeltaPositions[2] ) / 2.0;
            deltaX = encDeltaPositions[1] - deltaHeadingEnc * OFFSET_ENC_M_FROM_CENTER;

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            encGlobalPosition.add(deltaX * 1, deltaY * 1, 0);
        }

        public void updateGlobalVel(){

        }

        public void updateGlobalAccel(){

        }

        private void updatePosFromEncoders(boolean inTicks){//Обновляем позицию с датчиков
            encCurPositions[0] = encoderClass.getCurrentPosLeft();
            encCurPositions[1] = encoderClass.getCurrentPosMid();
            encCurPositions[2] = encoderClass.getCurrentPosRight();

            if(!inTicks){
                encCurPositions[0] = ticksToCm(encCurPositions[0]);
                encCurPositions[1] = ticksToCm(encCurPositions[1]);
                encCurPositions[2] = ticksToCm(encCurPositions[2]);
            }
            currentTime = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков

        }
        private void updateTimeToUpdPosFrmEnc(){//Время привязанное к обновлению позиции
            encDeltaPositions[0] = encCurPositions[0] - encLastPositions[0];
            encLastPositions[0] = encCurPositions[0];

            encDeltaPositions[1] = encCurPositions[1] - encLastPositions[1];
            encLastPositions[1] = encCurPositions[1];

            encDeltaPositions[2] = encCurPositions[2] - encLastPositions[2];
            encLastPositions[2] = encCurPositions[2];

            deltaTimes[0] = (currentTime - oldTimes[0])/1000;//время за которое программа доходит до сюда(время обновление программы)
            oldTimes[0] = currentTime;
        }
        private void updateVelFromEncoders(boolean inTicks){//Обновляем скорость с датчиков
            encCurVelocities[0] = encoderClass.getCurrentVelocityLeft();
            encCurVelocities[1] = encoderClass.getCurrentVelocityMid();
            encCurVelocities[2] = encoderClass.getCurrentVelocityRight();

            if(!inTicks){
                encCurVelocities[0] = encCurVelocities[0] / encoderClass.COUNTS_PER_CM;
                encCurVelocities[1] = encCurVelocities[1] / encoderClass.COUNTS_PER_CM;
                encCurVelocities[2] = encCurVelocities[2] / encoderClass.COUNTS_PER_CM;
            }
            currentTime = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков
        }

        private void updateTimeToUpdVelFrmEnc(){//Время привязанное к обновлению скорости
            encDeltaVelocities[0] = encCurVelocities[0] - encLastVelocities[0];
            encLastVelocities[0] = encCurVelocities[0];

            encDeltaVelocities[1] = encCurVelocities[1] - encLastVelocities[1];
            encLastVelocities[1] = encCurVelocities[1];

            encDeltaVelocities[2] = encCurVelocities[2] - encLastVelocities[2];
            encLastVelocities[2] = encCurVelocities[2];

            deltaTimes[1] = (currentTime - oldTimes[1])/1000;//время за которое программа доходит до сюда(время обновление программы)
            oldTimes[1] = currentTime;
        }


        private void updateEncodersVelocityNotCentric(){

            encodersVelNotCentric = new Vector2[]{
                    new Vector2(
                            0,
                            leftVelCur * 1),//0 - элемент - значение скорости для левого энкодера.
                    new Vector2(
                            midVelCur * 1,
                            0),          //1 - элемент - значение скорости для среднего энкодера.
                    new Vector2(
                            0,
                            rightVelCur * 1)//2 - элемент - значение скорости для правого энкодера.
            };

        }
        private void updateEncodersVelocityCentric(){

            encodersVelCentric = new Vector2[]{
                    new Vector2(
                            encodersVelNotCentric[0].length() * Math.cos(Math.toRadians(90) + yaw),
                            encodersVelNotCentric[0].length() * Math.sin(Math.toRadians(90) + yaw))//0 - элемент - значение скорости для левого энкодера.
                    ,
                    new Vector2(
                            encodersVelNotCentric[1].length() * Math.cos(yaw),
                            encodersVelNotCentric[1].length() * Math.sin(yaw))//1 - элемент - значение скорости для среднего энкодера.
                    ,
                    new Vector2(
                            encodersVelNotCentric[2].length() * Math.cos(Math.toRadians(90) + yaw),
                            encodersVelNotCentric[2].length() * Math.sin(Math.toRadians(90) + yaw))//2 - элемент - значение скорости для правого энкодера.
            };

        }

        private void updateRobotVelocityNotCentric(){
            robotSelfNotCentricVel = new Vector2(
                    encodersVelNotCentric[1].x * 1,
                    encodersVelNotCentric[0].y / 2.0 + encodersVelNotCentric[2].y / 2.0);
        }
        private void updateRobotVelocityCentric(){
            robotSelfCentricVel =  new Vector2(
                    encodersVelCentric[0].x / 2.0 + encodersVelCentric[1].x + encodersVelCentric[2].x / 2.0,
                    encodersVelCentric[0].y / 2.0 + encodersVelCentric[1].y + encodersVelCentric[2].y / 2.0);
        }

        private void updateEncodersAccelNotCentric(){

            encodersAccelNotCentric =  new Vector2[]{
                    new Vector2(0,
                            encodersVelNotCentric[0].y / deltaTimes[1]),

                    new Vector2(encodersVelNotCentric[1].x / deltaTimes[1],
                            0),

                    new Vector2(0,
                            encodersVelNotCentric[2].y / deltaTimes[1])

            };
        }

        private void updateEncodersAccelCentric(){
            encodersAccelCentric =  new Vector2[]{
                    new Vector2(encodersVelCentric[0].x / deltaTimes[1],
                            encodersVelCentric[0].y / deltaTimes[1]),

                    new Vector2(encodersVelCentric[1].x/ deltaTimes[1],
                            encodersVelCentric[1].y/ deltaTimes[1]),

                    new Vector2(encodersVelCentric[2].x / deltaTimes[1],
                            encodersVelCentric[2].x / deltaTimes[1])

            };
        }

        private void updateRobotAccelNotCentric(){
            robotSelfNotCentricAccel = new Vector2(
                    encodersAccelNotCentric[1].x * 1,
                    encodersAccelNotCentric[0].y / 2.0 + encodersVelNotCentric[2].y / 2.0);
        }
        private void updateRobotAccelCentric(){
            robotSelfCentricAccel = new Vector2(
                    encodersAccelCentric[0].x / 2.0 + encodersAccelCentric[1].x + encodersAccelCentric[2].x / 2.0,
                    encodersAccelCentric[0].y / 2.0 + encodersAccelCentric[1].y + encodersAccelCentric[2].y / 2.0);
        }

        private void updateRobotAngleVelCentric(){
            robotSelfAngleVelCentric = new Vector2(
                    gyro.imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate * 1,
                    gyro.imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate * 1);
        }

        private void updateRobotAngleAccelCentric(){
            robotSelfAngleVelCentric = new Vector2(
                    gyro.imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate * 1,
                    gyro.imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate * 1);
        }
    }

//        public void showMaxVelLeft(){
//            getMaxVelLeft();
//            telemetry.addLine("EncoderLeftMaxVel:")
//                    .addData("\nX", maxVelLeft.x)
//                    .addData("Y", maxVelLeft.y);
//        }
//        public void showMaxVelMid(){
//            getMaxVelMid();
//            telemetry.addLine("EncoderMidMaxVel:")
//                    .addData("\nX", maxVelMid.x)
//                    .addData("Y", maxVelMid.y);
//        }
//        public void showMaxVelRight(){
//            getMaxVelRight();
//            telemetry.addLine("EncoderRightMaxVel:")
//                    .addData("\nX", maxVelRight.x)
//                    .addData("Y", maxVelRight.y);
//        }
//
//        public void showMaxAccelLeft(){
//            getMaxAccelLeft();
//            telemetry.addLine("EncoderLeftMaxAccel:")
//                    .addData("\nX", maxAccelLeft.x)
//                    .addData("Y", maxAccelLeft.y);
//        }
//        public void showMaxAccelMid(){
//            getMaxAccelMid();
//            telemetry.addLine("EncoderMidMaxAccel:")
//                    .addData("\nX", maxAccelMid.x)
//                    .addData("Y", maxAccelMid.y);
//        }
//        public void showMaxAccelRight(){
//            getMaxAccelRight();
//            telemetry.addLine("EncoderRightMaxAccel:")
//                    .addData("\nX", maxAccelRight.x)
//                    .addData("Y", maxAccelRight.y);
//        }
}