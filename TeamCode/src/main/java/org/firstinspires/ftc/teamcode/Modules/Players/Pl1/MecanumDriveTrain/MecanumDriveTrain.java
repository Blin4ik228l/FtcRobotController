package org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MathUtils.Position;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.Modules.Module;

import java.util.concurrent.TimeUnit;

public class MecanumDriveTrain extends Module {
    //Телега робота(моторы + колёса) с энкодерами.

    public MecanumDriveTrain(OpMode op){
        super(op.telemetry);

        gyro = new GyroSkope(op);
//        odometry = new Odometry(op);
        exOdometry = new ExOdometry(op);

        rightB = op.hardwareMap.get(DcMotor.class, "rightB");
        rightF = op.hardwareMap.get(DcMotor.class, "rightF");
        leftB = op.hardwareMap.get(DcMotor.class, "leftB");
        leftF = op.hardwareMap.get(DcMotor.class, "leftF");

        rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightB.setDirection(DcMotorSimple.Direction.FORWARD);
        rightF.setDirection(DcMotorSimple.Direction.FORWARD);
        leftB.setDirection(DcMotorSimple.Direction.REVERSE);
        leftF.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Drivetrain Inited");
    }
   public DcMotor leftB;
   public DcMotor leftF;
   public DcMotor rightB;
   public DcMotor rightF;
//   public Odometry odometry;
   public GyroSkope gyro;

   public ExOdometry exOdometry;
    public void setPower(double yVol, double xVol, double angVol, double vyrVol){
        //движение по y - это вперёд - назад
        //движение по x - это влево - вправо
        //angVol поворот
        exOdometry.updateAll();//Обноволяем одометрию постоянно когда вызываем метод setPower

        leftF.setPower(yVol + xVol + angVol + vyrVol);
        leftB.setPower(yVol - xVol + angVol + vyrVol);
        rightF.setPower(yVol - xVol - angVol - vyrVol);
        rightB.setPower(yVol + xVol - angVol - vyrVol);
    }
   public class GyroSkope {
       public GyroSkope(OpMode op){
           // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
           // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
           // and named "imu".
           imu = op.hardwareMap.get(IMU.class, "imu");
           parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                   RevHubOrientationOnRobot.LogoFacingDirection.UP,
                   RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
           ));

           imu.initialize(parameters);

           imu.resetYaw();
       }
       public Deadline imuResetTime = new Deadline(500, TimeUnit.MILLISECONDS);
       IMU.Parameters parameters;
       public IMU imu;
       public void getYaw(){
           telemetry.addLine("Gyro")
                   .addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
           telemetry.addLine();
       }
   }

    public class ExOdometry{
        //Все энкодеры на телеге составляющие общую систему оценки положения робота в пространстве.

        public ExOdometry(OpMode op){
//            voltageSensor = op.hardwareMap.get(VoltageSensor.class, "");

            this.globalPosition = new Position();
            this.deltaPosition = new Position();

            encLeft = op.hardwareMap.get(DcMotorEx.class, "leftB");
            encRight = op.hardwareMap.get(DcMotorEx.class, "rightB");
            encMid = op.hardwareMap.get(DcMotorEx.class, "rightF" );

            encRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем правый энкодер
            encLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем левый энкодер
            encMid.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем средний энкодер

            encMid.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем средний энкодер
            encRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем правый энкодер
            encLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            runTimeL = new ElapsedTime();
            runTimeM = new ElapsedTime();
            runTimeR = new ElapsedTime();
            dT = new double[4];
            oldTime = new double[4];

            for (int i = 0; i < 4; i++)
            {
                dT[i] = 0;
                oldTime[i] = 0;
            }
        }
        public double COUNTS_PER_ENCODER_REV = 2000;
        public double DRIVE_GEAR_REDUCTION = 1;
        public double ENC_WHEEL_DIAM_CM = 4.8;
        public double COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
                (ENC_WHEEL_DIAM_CM * Math.PI);

        public final ElapsedTime runTimeL;                                      // Пройденное время
        public final ElapsedTime runTimeR;
        public final ElapsedTime runTimeM;
        public final double[] oldTime;                                         // Предыдущее время
        public final double[] dT;
        public final Position deltaPosition;                                   // Относительное перемещение
        public final Position globalPosition;
        public double encLOld, encROld, encMOld;
        public VoltageSensor voltageSensor;
        public DcMotorEx encLeft;
        public DcMotorEx encMid;
        public DcMotorEx encRight;
        public Vector2 maxVelLeft = new Vector2();
        public Vector2 maxVelRight = new Vector2();
        public Vector2 maxVelMid = new Vector2();

        public Vector2[] maxVels = new Vector2[3];

        public Vector2 maxAccelLeft = new Vector2();
        public Vector2 maxAccelRight = new Vector2();
        public Vector2 maxAccelMid = new Vector2();
        private double ticksToCm(double ticks){
            return ticks / COUNTS_PER_CM;
        }

        public void updateAll(){
            updateGlobalPosition();
        }
        private void updateGlobalPosition(){
            double leftEncoderYNow = ticksToCm(encLeft.getCurrentPosition() );
            double deltaLeftEncoderY = leftEncoderYNow - encLOld;
            encLOld = leftEncoderYNow;

            double rightEncoderYNow = ticksToCm(encRight.getCurrentPosition() );
            double deltaRightEncoderY = rightEncoderYNow - encROld;
            encROld = rightEncoderYNow;

            double encoderXNow = ticksToCm(encMid.getCurrentPosition());
            double deltaEncoderX = encoderXNow - encMOld;
            encMOld = encoderXNow;

            // Если перемещения не было - выходим из метода
            if(deltaLeftEncoderY == 0 && deltaRightEncoderY == 0 && deltaEncoderX == 0 ) {
                return;
            }

            // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
            // Для корректной работы этот метод должен работать в непрерывном цикле
            double deltaRad = -(deltaRightEncoderY - deltaLeftEncoderY) / DIST_BETWEEN_ENC_X;
            double deltaY = -(deltaLeftEncoderY + deltaRightEncoderY ) / 2.0;
            double deltaX = -deltaEncoderX - deltaRad * OFFSET_ENC_M_FROM_CENTER;

            deltaPosition.setX(deltaX);
            deltaPosition.setY(deltaY);
            deltaPosition.setHeading(deltaRad);

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            globalPosition.add(deltaPosition.toVector(), deltaPosition.getHeading());
        }

        public Vector2 getMaxVelLeft() {
            if(maxVelLeft.length() < getEncLeftVelocity(false).length()) maxVelLeft = getEncLeftVelocity(false);
            return maxVelLeft;
        }

        public Vector2 getMaxVelRight() {
            if(maxVelRight.length() < getEncRightVelocity(false).length()) maxVelRight = getEncRightVelocity(false);
            return maxVelRight;
        }

        public Vector2 getMaxVelMid() {
            if(maxVelMid.length() < getEncMidVelocity(false).length()) maxVelMid = getEncMidVelocity(false);
            return maxVelMid;
        }

        public Vector2 getMaxAccelLeft() {
            if(maxAccelLeft.length() < getEncLeftAccel(false).length()) maxAccelLeft = getEncLeftAccel(false);
            return maxAccelLeft;
        }

        public Vector2 getMaxAccelRight() {
            if(maxAccelRight.length() < getEncRightAccel(false).length()) maxAccelRight = getEncRightAccel(false);
            return maxAccelRight;
        }

        public Vector2 getMaxAccelMid() {
            if(maxAccelMid.length() < getEncMidAccel(false).length()) maxAccelMid = getEncMidAccel(false);
            return maxAccelMid;
        }

        public Vector2[] getEncodersVelocityNotCentric(boolean inTicks){
            double velLeft = -encLeft.getVelocity();
            double velMid = -encMid.getVelocity();
            double velRight = -encRight.getVelocity();

            if(!inTicks){
                velLeft = (velLeft / COUNTS_PER_ENCODER_REV) * Math.PI * ENC_WHEEL_DIAM_CM;
                velMid = (velMid / COUNTS_PER_ENCODER_REV) * Math.PI * ENC_WHEEL_DIAM_CM;
                velRight = (velRight / COUNTS_PER_ENCODER_REV) * Math.PI * ENC_WHEEL_DIAM_CM;
            }

            return new Vector2[]{
                    new Vector2(
                            0,
                            velLeft * 1),//0 - элемент - значение скорости для левого энкодера.
                    new Vector2(
                            velMid * 1,
                            0),          //1 - элемент - значение скорости для среднего энкодера.
                    new Vector2(
                            0,
                            velRight * 1)//2 - элемент - значение скорости для правого энкодера.
            };

        }
        public Vector2[] getEncodersVelocityCentric(boolean inTicks){

            return new Vector2[]{
                    new Vector2(
                            getEncodersVelocityNotCentric(inTicks)[0].length() * Math.cos(Math.toRadians(90) + gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                            getEncodersVelocityNotCentric(inTicks)[0].length() * Math.sin(Math.toRadians(90) + gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)))//0 - элемент - значение скорости для левого энкодера.
                    ,
                    new Vector2(
                            getEncodersVelocityNotCentric(inTicks)[1].length() * Math.cos(gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                            getEncodersVelocityNotCentric(inTicks)[1].length() * Math.sin(gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)))//1 - элемент - значение скорости для среднего энкодера.
                    ,
                    new Vector2(
                            getEncodersVelocityNotCentric(inTicks)[2].length() * Math.cos(Math.toRadians(90) + gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                            getEncodersVelocityNotCentric(inTicks)[2].length() * Math.sin(Math.toRadians(90) + gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)))//2 - элемент - значение скорости для правого энкодера.
            };

        }

        public Vector2 getRobotVelocityNotCentric(boolean inTicks){
            return new Vector2(
                     getEncodersVelocityNotCentric(inTicks)[1].x * 1,
                    getEncodersVelocityNotCentric(inTicks)[0].y / 2.0 + getEncodersVelocityNotCentric(inTicks)[2].y / 2.0);
        }
        public Vector2 getRobotVelocityCentric(boolean inTicks){
            return new Vector2(
                    getEncodersVelocityCentric(inTicks)[0].x / 2.0 + getEncodersVelocityCentric(inTicks)[1].x + getEncodersVelocityCentric(inTicks)[2].x / 2.0,
                    getEncodersVelocityCentric(inTicks)[0].y / 2.0 + getEncodersVelocityCentric(inTicks)[1].y + getEncodersVelocityCentric(inTicks)[2].y / 2.0);
        }
        public Vector2 getEncLeftVelocity(boolean inTicks){
            double vel = -encLeft.getVelocity();

            if(!inTicks){
                vel = (vel / COUNTS_PER_ENCODER_REV) * Math.PI * ENC_WHEEL_DIAM_CM;
            }

            return new Vector2(
                    vel * Math.cos(Math.toRadians(90) + gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                    vel * Math.sin(Math.toRadians(90) + gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        }

        public Vector2 getEncMidVelocity(boolean inTicks){
            double vel = -encMid.getVelocity();

            if(!inTicks){
                vel = (vel / COUNTS_PER_ENCODER_REV) * Math.PI * ENC_WHEEL_DIAM_CM;
            }

            return new Vector2(
                    vel * Math.cos(gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                    vel * Math.sin(gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        }
        public Vector2 getEncRightVelocity(boolean inTicks){
            double vel = -encLeft.getVelocity();

            if(!inTicks){
                vel = (vel / COUNTS_PER_ENCODER_REV) * Math.PI * ENC_WHEEL_DIAM_CM;
            }

            return new Vector2(
                    vel * Math.cos(Math.toRadians(90) + gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                    vel * Math.sin(Math.toRadians(90) + gyro.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        }

        public Vector2 getEncLeftAccel(boolean inTicks){
            dT[0] = (runTimeL.milliseconds() - oldTime[0])/1000;
            oldTime[0] = runTimeL.milliseconds();

            return new Vector2(
                    getEncLeftVelocity(inTicks).x / dT[0],
                    getEncLeftVelocity(inTicks).y / dT[0]);
        }

        public Vector2 getEncMidAccel(boolean inTicks){
            dT[1] = (runTimeM.milliseconds() - oldTime[1])/1000;
            oldTime[1] = runTimeM.milliseconds();

            return new Vector2(
                    getEncMidVelocity(inTicks).x / dT[1],
                    getEncMidVelocity(inTicks).y / dT[1]);
        }

        public Vector2 getEncRightAccel(boolean inTicks){
            dT[2] = (runTimeR.milliseconds() - oldTime[2])/1000;
            oldTime[2] = runTimeR.milliseconds();

            return new Vector2(
                    getEncRightVelocity(inTicks).x / dT[2],
                    getEncRightVelocity(inTicks).y / dT[2]);
        }

        public void showEncodersVelocityComponents(){
            telemetry.addLine("Encoders Velo:")
                    .addData("\nLeft encoder X", getEncLeftVelocity(false).x)
                    .addData("\nLeft encoder Y", getEncLeftVelocity(false).y)
                    .addData("\nMid encoder X", getEncMidVelocity(false).x)
                    .addData("\nMid encoder X", getEncMidVelocity(false).y)
                    .addData("\nRight encoder X", getEncRightVelocity(false).x)
                    .addData("\nRight encoder Y", getEncRightVelocity(false).y);
            telemetry.addLine();
        }

        public void showMaxVelLeft(){
            getMaxVelLeft();
            telemetry.addLine("EncoderLeftMaxVel:")
                    .addData("\nX", maxVelLeft.x)
                    .addData("Y", maxVelLeft.y);
        }
        public void showMaxVelMid(){
            getMaxVelMid();
            telemetry.addLine("EncoderMidMaxVel:")
                    .addData("\nX", maxVelMid.x)
                    .addData("Y", maxVelMid.y);
        }
        public void showMaxVelRight(){
            getMaxVelRight();
            telemetry.addLine("EncoderRightMaxVel:")
                    .addData("\nX", maxVelRight.x)
                    .addData("Y", maxVelRight.y);
        }

        public void showMaxAccelLeft(){
            getMaxAccelLeft();
            telemetry.addLine("EncoderLeftMaxAccel:")
                    .addData("\nX", maxAccelLeft.x)
                    .addData("Y", maxAccelLeft.y);
        }
        public void showMaxAccelMid(){
            getMaxAccelMid();
            telemetry.addLine("EncoderMidMaxAccel:")
                    .addData("\nX", maxAccelMid.x)
                    .addData("Y", maxAccelMid.y);
        }
        public void showMaxAccelRight(){
            getMaxAccelRight();
            telemetry.addLine("EncoderRightMaxAccel:")
                    .addData("\nX", maxAccelRight.x)
                    .addData("Y", maxAccelRight.y);
        }
    }
}
