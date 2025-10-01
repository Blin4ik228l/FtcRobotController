package org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
        odometry = new Odometry(op);

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
   public Odometry odometry;
   public GyroSkope gyro;

    public void setPower(double yVol, double xVol, double angVol, double vyrVol){
        //движение по y - это вперёд - назад
        //движение по x - это влево - вправо
        //angVol поворот
        odometry.updateAll();//Обноволяем одометрию постоянно когда вызываем метод setPower

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

//       public void update(){
//           imu.startAccelerationIntegration(new org.firstinspires.ftc.robotcore.external.navigation.Position(), new Velocity(), 1000);
//       }

       public void getYaw(){
           telemetry.addLine("Gyro")
                   .addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
           telemetry.addLine();
       }
       public void composeTelemetry() {

//           // В начале каждого обновления телеметрии соберирает набор данных
//           // из IMU, которые мы затем отобразим в отдельных строках.
//           telemetry.addAction(new Runnable() { @Override public void run()
//           {
//               // Acquiring the angles is relatively expensive; we don't want
//               // to do that in each of the three items that need that info, as that's
//               // three times the necessary expense.
//               angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//               gravity  = imu.getGravity();
//           }
//           });
//
//           telemetry.addLine()
//                   .addData("status", new Func<String>() {
//                       @Override public String value() {
//                           return imu.getSystemStatus().toShortString();
//                       }
//                   })
//                   .addData("calib", new Func<String>() {
//                       @Override public String value() {
//                           return imu.getCalibrationStatus().toString();
//                       }
//                   });

//           telemetry.addLine()
//                   .addData("heading", new Func<String>() {
//                       @Override public String value() {
//                           return formatAngle(angles.angleUnit, angles.firstAngle);
//                       }
//                   })
//                   .addData("roll", new Func<String>() {
//                       @Override public String value() {
//                           return formatAngle(angles.angleUnit, angles.secondAngle);
//                       }
//                   })
//                   .addData("pitch", new Func<String>() {
//                       @Override public String value() {
//                           return formatAngle(angles.angleUnit, angles.thirdAngle);
//                       }
//                   });
//
//           telemetry.addLine()
//                   .addData("grvty", new Func<String>() {
//                       @Override public String value() {
//                           return gravity.toString();
//                       }
//                   })
//                   .addData("mag", new Func<String>() {
//                       @Override public String value() {
//                           return String.format(Locale.getDefault(), "%.3f",
//                                   Math.sqrt(gravity.xAccel*gravity.xAccel
//                                           + gravity.yAccel*gravity.yAccel
//                                           + gravity.zAccel*gravity.zAccel));
//                       }
//                   });
       }

       //----------------------------------------------------------------------------------------------
       // Formatting
       //----------------------------------------------------------------------------------------------
//
//       String formatAngle(AngleUnit angleUnit, double angle) {
//           return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//       }
//
//       String formatDegrees(double degrees){
//           return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//       }
   }

    public class Odometry{
        //Все энкодеры на телеге составляющие общую систему оценки положения робота в пространстве.

        public double COUNTS_PER_ENCODER_REV = 2000;
        public double DRIVE_GEAR_REDUCTION = 1;
        public double ENC_WHEEL_DIAM_CM = 4.8;
        public double COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
                (ENC_WHEEL_DIAM_CM * Math.PI);
        public  Odometry (OpMode op){
            this.globalPosition = new Position();
            this.deltaPosition = new Position();

            this.velocity = new Vector2(0,0);
            this.oldVelocity = new Vector2(0,0);
            this.acceleration = new Vector2(0,0);

            runtime = new ElapsedTime();
            dt = new double[4];
            oldTime = new double[4];

            for (int i = 0; i < 4; i++)
            {
                dt[i] = 0;
                oldTime[i] = 0;
            }

            encM = op.hardwareMap.get(DcMotorEx.class, "rightF");
            encL = op.hardwareMap.get(DcMotorEx.class, "leftB");
            encR = op.hardwareMap.get(DcMotorEx.class, "rightB");

            encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем правый энкодер
            encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем левый энкодер
            encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем средний энкодер

            encM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем средний энкодер
            encR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем правый энкодер
            encL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем левый энкодер

            telemetry.addLine("Odometry Inited");                             // объявляем об обнавлении одометрии
        }
        private final ElapsedTime runtime;                                      // Пройденное время
        private final double[] oldTime;                                         // Предыдущее время
        private final double[] dt;                                              // Разница во времени
        private double encLOld, encROld, encMOld;                               // Значения энкодера на предыдущем шаге
        private double angularVelocity, angularAcceleration, oldAngularVelocity;
        private DcMotorEx encM;                                                 // Объекты энкодеров
        private DcMotorEx encL;                                                 //
        private DcMotorEx encR;                                                 //
        private final Position deltaPosition;                                   // Относительное перемещение
        private final Position globalPosition;                                  // Глобальное положение
        private Position startGlobalPosition = new Position();
        private final Vector2 velocity, oldVelocity, acceleration;              // Вектора скорость и ускорение ОТНОСИТЕЛЬНО КООРДИНАТ РОБОТА
        private double maxAcceleration, maxVel;                                 // Максимальные скорость и ускорение

        public void updateAll(){
            if(updateGlobalPosition()){//Возвраем true/false для того чтобы понять: есть ли смысл дальше считать или нет?
                updateVelocity();
                updateAcceleration();
                updateAngularVelocity();
                updateAngularAcceleration();
            }
        }

        private Position getStartGlobalPosition() {
            return startGlobalPosition;
        }
        private DcMotorEx getEncL() {                                   // создаем метод для получения левого энкодера
            return encL;
        }
        private DcMotorEx getEncM() {                                   // создаем метод для получения левого энкодера
            return encM;
        }
        private DcMotorEx getEncR() {                                   // создаем метод для получения левого энкодера
            return encR;
        }

        private double[] getDt() {                                      // создаем метод для получения времени на цикл
            return dt;
        }

        private double getMaxVel(){                                     // создаем метод для получения максимальной скорости
            return maxVel;
        }
        private double getMaxAcceleration(){                            // создаем метод для получения максимального ускорения
            return maxAcceleration;
        }

        public Position getGlobalPosition(){                           // создаем метод для получения глобальных координат
            return globalPosition;
        }

        public Vector2 getVelocity(){                                  // создаем метод для получения вектора скорости
            return velocity;
        }

        // Геттер вектора ускорения
        private Vector2 getAcceleration(){
            return acceleration;
        }
        // Геттер углового вектора ускорения
        private double getAngularAcceleration(){
            return angularAcceleration;
        }
        // Геттер углового вектора ускорения
        private double getAngularVelocity(){
            return angularVelocity;
        }
        // Геттер углового вектора ускорения
        private double getSpeed(){
            return velocity.length();
        }

        // Тики энкодера в сантиметры
        private double ticksToCm(double ticks){
            return ticks / COUNTS_PER_CM;
        }

        private double cmToTicks(double cm){
            return cm * COUNTS_PER_CM;
        }

        private void setGlobalPosition(Position position) {
            globalPosition.setX(position.getX());
            globalPosition.setY(position.getY());
            globalPosition.setHeading(position.getHeading());
        }

        private void updateMaxAcceleration(){
            if(Math.abs(acceleration.length()) > Math.abs(maxAcceleration)){
                maxAcceleration = acceleration.length();
            }
        }

        private void updateMaxVel(){
            if(Math.abs(velocity.length()) > Math.abs(maxVel)){
                maxVel = velocity.length();
            }
        }

        // Обновление вектора скорости робота
        private void updateVelocity(){
            dt[0] = (runtime.milliseconds() - oldTime[0])/1000.0;// считаем время одного цикла
            oldTime[0] = runtime.milliseconds();

            oldVelocity.x = velocity.x;
            oldVelocity.y = velocity.y;

            velocity.x = (deltaPosition.toVector().x)/dt[0];
            velocity.y = (deltaPosition.toVector().y)/dt[0];
        }
        private double getDeltaVel(){
            return velocity.x - velocity.y;
        }
        // Обновление вектора ускорения робота
        private void updateAcceleration() {
            dt[1] = (runtime.milliseconds() - oldTime[1])/1000.0;
            oldTime[1] = runtime.milliseconds();

            acceleration.x = (velocity.x - oldVelocity.x)/dt[1];
            acceleration.y = (velocity.y - oldVelocity.y)/dt[1];
        }
        // Обнавление вектора угловой скорости
        private void updateAngularVelocity(){
            dt[3] = (runtime.milliseconds() - oldTime[3])/1000.0;
            oldTime[3] = runtime.milliseconds();

            oldAngularVelocity = angularVelocity;
            angularVelocity = deltaPosition.getHeading()/dt[3];
        }
        // Обновление вектора углового ускорения робота
        private void updateAngularAcceleration() {
            dt[2] = (runtime.milliseconds() - oldTime[2])/1000.0;
            oldTime[2] = runtime.milliseconds();

            angularAcceleration = (angularVelocity - oldAngularVelocity)/dt[2];
        }
        // Обновление положения робота на поле с помощью следящих колес
        private boolean updateGlobalPosition(){
            double leftEncoderYNow = ticksToCm(encL.getCurrentPosition() );
            double deltaLeftEncoderY = leftEncoderYNow - encLOld;
            encLOld = leftEncoderYNow;

            double rightEncoderYNow = ticksToCm(encR.getCurrentPosition() );
            double deltaRightEncoderY = rightEncoderYNow - encROld;
            encROld = rightEncoderYNow;

            double encoderXNow = ticksToCm(encM.getCurrentPosition());
            double deltaEncoderX = encoderXNow - encMOld;
            encMOld = encoderXNow;

            // Если перемещения не было - выходим из метода
            if(deltaLeftEncoderY == 0 && deltaRightEncoderY == 0 && deltaEncoderX == 0 ) {
                return false;
            }

            // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
            // Для корректной работы этот метод должен работать в непрерывном цикле
            double deltaRad = -(deltaRightEncoderY - deltaLeftEncoderY) / DIST_BETWEEN_ENC_X;
            double deltaY = -(deltaLeftEncoderY + deltaRightEncoderY ) / 2.0;
            double deltaX = -deltaEncoderX - deltaRad * OFFSET_ENC_M_FROM_CENTER;


//            deltaPosition.add(deltaX, deltaY, deltaRad);

            deltaPosition.setX(deltaX);
            deltaPosition.setY(deltaY);
            deltaPosition.setHeading(deltaRad);

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            globalPosition.add(deltaPosition.toVector(), deltaPosition.getHeading());

            return true;
        }

        // Вывод позиции робота
        public void getRobotPos(){
            telemetry.addLine("Robot position")
                    .addData("\nX:", globalPosition.getX())
                    .addData("\nY", globalPosition.getY())
                    .addData("\nHeading", globalPosition.getHeading() * (180/Math.PI));
            telemetry.addLine();
        }

//        public void getRobotPos(){
//            telemetry.addLine("Robot position")
//                    .addData("\nX:", deltaPosition.getX())
//                    .addData("\nY", deltaPosition.getY())
//                    .addData("\nHeading", deltaPosition.getHeading() * (180/Math.PI));
//            telemetry.addLine();
//        }
        // Вывод позиции энкодеров
        public void getEncPos(){
            telemetry.addLine("Encoders statements")
                    .addData("\nEncL", encL.getCurrentPosition())
                    .addData("\nEncM", encM.getCurrentPosition())
                    .addData("\nEncR", encR.getCurrentPosition());
            telemetry.addLine();
        }
    }
}
