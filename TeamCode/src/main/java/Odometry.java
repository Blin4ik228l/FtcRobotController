import static org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts.DIST_BETWEEN_ENC_X;
import static org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts.OFFSET_ENC_M_FROM_CENTER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;

public class Odometry{
    //Все энкодеры на телеге составляющие общую систему оценки положения робота в пространстве.

    public double COUNTS_PER_ENCODER_REV = 2000;
    public double DRIVE_GEAR_REDUCTION = 1;
    public double ENC_WHEEL_DIAM_CM = 4.8;
    public double COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
            (ENC_WHEEL_DIAM_CM * Math.PI);
    public  Odometry (OpMode op){
        this.globalPosition2D = new Position2D();
        this.deltaPosition2D = new Position2D();

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

//        telemetry.addLine("Odometry Inited");                             // объявляем об обнавлении одометрии
    }
    private final ElapsedTime runtime;                                      // Пройденное время
    private final double[] oldTime;                                         // Предыдущее время
    private final double[] dt;                                              // Разница во времени
    private double encLOld, encROld, encMOld;                               // Значения энкодера на предыдущем шаге
    private double angularVelocity, angularAcceleration, oldAngularVelocity;
    public DcMotorEx encM;                                                 // Объекты энкодеров
    public DcMotorEx encL;                                                 //
    public DcMotorEx encR;                                                 //
    private final Position2D deltaPosition2D;                                   // Относительное перемещение
    private final Position2D globalPosition2D;                                  // Глобальное положение
    private Position2D startGlobalPosition2D = new Position2D();
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

    private Position2D getStartGlobalPosition() {
        return startGlobalPosition2D;
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

    public Position2D getGlobalPosition(){                           // создаем метод для получения глобальных координат
        return globalPosition2D;
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

    private void setGlobalPosition(Position2D position2D) {
        globalPosition2D.setX(position2D.getX());
        globalPosition2D.setY(position2D.getY());
        globalPosition2D.setHeading(position2D.getHeading());
    }

    // Обновление вектора скорости робота
    private void updateVelocity(){
        dt[0] = (runtime.milliseconds() - oldTime[0])/1000.0;// считаем время одного цикла
        oldTime[0] = runtime.milliseconds();

        oldVelocity.x = velocity.x;
        oldVelocity.y = velocity.y;

        velocity.x = (deltaPosition2D.toVector().x)/dt[0];
        velocity.y = (deltaPosition2D.toVector().y)/dt[0];
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
        angularVelocity = deltaPosition2D.getHeading()/dt[3];
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

        deltaPosition2D.setX(deltaX);
        deltaPosition2D.setY(deltaY);
        deltaPosition2D.setHeading(deltaRad);

        // Векторный поворот и добавление глобального перемещения к глобальным координатам
        globalPosition2D.add(deltaPosition2D.toVector(), deltaPosition2D.getHeading());

        return true;
    }

//    // Вывод позиции робота
//    public void getRobotPos(){
//        telemetry.addLine("Robot position")
//                .addData("\nX:", globalPosition.getX())
//                .addData("\nY", globalPosition.getY())
//                .addData("\nHeading", globalPosition.getHeading() * (180/Math.PI));
//        telemetry.addLine();
//    }
//
//
//    // Вывод позиции энкодеров
//    public void getEncPos(){
//        telemetry.addLine("Encoders statements")
//                .addData("\nEncL", encL.getCurrentPosition())
//                .addData("\nEncM", encM.getCurrentPosition())
//                .addData("\nEncR", encR.getCurrentPosition());
//        telemetry.addLine();
//    }
}
