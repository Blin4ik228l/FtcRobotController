package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.OtherStates.EncoderStatus;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

// Отдельный класс, работающий с одометрией в отдельном потоке
public class Odometry extends Thread implements Module {
    private final OpMode op;

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
    private final Position startGlobalPosition;
    private final Vector2 velocity, oldVelocity, acceleration;              // Вектора скорость и ускорение ОТНОСИТЕЛЬНО КООРДИНАТ РОБОТА
    private double maxAcceleration, maxVel;                                 // Максимальные скорость и ускорение

    private EncoderStatus encXst;
    private EncoderStatus encYst;
    private EncoderStatus encRadSt;

    public  Odometry (Position startPosition, OpMode op){
        this.op = op;

        startGlobalPosition = startPosition;
        this.globalPosition = new Position(startPosition);
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
    }

    public  Odometry (OpMode op){
        this(new Position(0,0,0), op);                            // обнуляем позицию
    }

    @Override
    public void init() {
      encM = op.hardwareMap.get(DcMotorEx.class, "encM");
        encL = op.hardwareMap.get(DcMotorEx.class, "encL");
        encR = op.hardwareMap.get(DcMotorEx.class, "encR");
        encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем правый энкодер
        encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем левый энкодер
        encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем средний энкодер

        encM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем средний энкодер
        encR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем правый энкодер
        encL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем левый энкодер

        this.setDaemon(true);
        this.start();

        op.telemetry.addLine("Odometry Inited");                             // объявляем об обнавлении одометрии
    }

    @Override                                                                   // Запускаем методы для обновления данных,
                                                                                // пока объект существует
    public void run() {
        while (this.isAlive()){
            updateGlobalPosition();

            updateVelocity();
            updateAcceleration();
            updateAngularAcceleration();
            updateAngularVelocity();
        }
    }

    public Position getStartGlobalPosition() {
        return startGlobalPosition;
    }

    public EncoderStatus getEncXst() {
        return encXst;
    }

    public EncoderStatus getEncYst() {
        return encYst;
    }

    public EncoderStatus getEncRadSt() {
        return encRadSt;
    }

    public synchronized DcMotorEx getEncL() {                                   // создаем метод для получения левого энкодера
        return encL;
    }
    public synchronized DcMotorEx getEncM() {                                   // создаем метод для получения левого энкодера
        return encM;
    }
    public synchronized DcMotorEx getEncR() {                                   // создаем метод для получения левого энкодера
        return encR;
    }


    public synchronized double[] getDt() {                                      // создаем метод для получения времени на цикл
        return dt;
    }

    public synchronized double getMaxVel(){                                     // создаем метод для получения максимальной скорости
        return maxVel;
    }
    public synchronized double getMaxAcceleration(){                            // создаем метод для получения максимального ускорения
        return maxAcceleration;
    }

    public synchronized Position getGlobalPosition(){                           // создаем метод для получения глобальных координат
        return globalPosition;
    }

    public synchronized Vector2 getVelocity(){                                  // создаем метод для получения вектора скорости
        return velocity;
    }

    // Геттер вектора ускорения
    public synchronized Vector2 getAcceleration(){
        return acceleration;
    }
    // Геттер углового вектора ускорения
    public synchronized double getAngularAcceleration(){
        return angularAcceleration;
    }
    // Геттер углового вектора ускорения
    public synchronized double getAngularVelocity(){
        return angularVelocity;
    }
    // Геттер углового вектора ускорения
    public synchronized double getSpeed(){
        return velocity.length();
    }

    // Тики энкодера в сантиметры
    public synchronized double ticksToCm(double ticks){
        return ticks / Consts.TICK_PER_CM;
    }

    public synchronized double cmToTicks(double cm){
        return cm * Consts.TICK_PER_CM;
    }

    public synchronized void setGlobalPosition(Position position) {
        globalPosition.setX(position.getX());
        globalPosition.setY(position.getY());
        globalPosition.setHeading(position.getHeading());
    }

    private synchronized void updateMaxAcceleration(){
        if(Math.abs(acceleration.length()) > Math.abs(maxAcceleration)){
            maxAcceleration = acceleration.length();
        }
    }

    private synchronized void updateMaxVel(){
        if(Math.abs(velocity.length()) > Math.abs(maxVel)){
            maxVel = velocity.length();
        }
    }

    // Обновление вектора скорости робота
    private synchronized void updateVelocity(){
        dt[0] = (runtime.milliseconds() - oldTime[0])/1000.0;// считаем время одного цикла
        oldTime[0] = runtime.milliseconds();

        oldVelocity.x = velocity.x;
        oldVelocity.y = velocity.y;

        velocity.x = (deltaPosition.toVector().x)/dt[0];
        velocity.y = (deltaPosition.toVector().y)/dt[0];

    }
    public synchronized double getDeltaVel(){
        return velocity.x - velocity.y;
    }
    // Обновление вектора ускорения робота
    private synchronized void updateAcceleration() {
        dt[1] = (runtime.milliseconds() - oldTime[1])/1000.0;
        oldTime[1] = runtime.milliseconds();


        acceleration.x = (velocity.x - oldVelocity.x)/dt[1];
        acceleration.y = (velocity.y - oldVelocity.y)/dt[1];
    }
    // Обнавление вектора угловой скорости
    private synchronized void updateAngularVelocity(){
        dt[3] = (runtime.milliseconds() - oldTime[3])/1000.0;
        oldTime[3] = runtime.milliseconds();

        oldAngularVelocity = angularVelocity;
        angularVelocity = deltaPosition.getHeading()/dt[3];
    }
    // Обновление вектора углового ускорения робота
    private synchronized void updateAngularAcceleration() {
        dt[2] = (runtime.milliseconds() - oldTime[2])/1000.0;
        oldTime[2] = runtime.milliseconds();

        angularAcceleration = (angularVelocity - oldAngularVelocity)/dt[2];
    }
    // Обновление положения робота на поле с помощью следящих колес
    private synchronized void updateGlobalPosition(){
        double leftEncoderXNow = ticksToCm(-encL.getCurrentPosition() );
        double deltaLeftEncoderX = leftEncoderXNow - encLOld;
        encLOld = leftEncoderXNow;

        double rightEncoderXNow = ticksToCm(encR.getCurrentPosition() );
        double deltaRightEncoderX = rightEncoderXNow - encROld;
        encROld = rightEncoderXNow;

        double encoderYNow = ticksToCm(encM.getCurrentPosition());
        double deltaEncoderY = encoderYNow - encMOld;
        encMOld = encoderYNow;

        // Если перемещения не было - выходим из метода
        if(deltaLeftEncoderX == 0 && deltaRightEncoderX == 0 && deltaEncoderY == 0 ) {
            encXst = EncoderStatus.ZeroDelta;
            encYst = EncoderStatus.ZeroDelta;
            encRadSt = EncoderStatus.ZeroDelta;
            return;
        }

        // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
        // Для корректной работы этот метод должен работать в непрерывном цикле
        double deltaRad = (-(deltaRightEncoderX + deltaLeftEncoderX) / Consts.DIST_BETWEEN_ENC_X);
        double deltaX = (deltaLeftEncoderX - deltaRightEncoderX ) / 2.0;
        double deltaY = (deltaEncoderY) - deltaRad * Consts.OFFSET_ENC_M_FROM_CENTER;

        //TODO: понять каково перемещение по энкодеру при застревании, на сколько оно маленькое
        if(deltaX == 0) encXst = EncoderStatus.ZeroDelta;
        else if(Math.abs(deltaX) < 0.05) encXst = EncoderStatus.SmallDelta;
        else encXst = EncoderStatus.InMoving;

        if(deltaY == 0) encYst = EncoderStatus.ZeroDelta;
        //Нужно понимать, что значение у центрального может быть маленьким при кручении
        else if(Math.abs(deltaY) < 0.05 && Math.abs(deltaRad) == 0 || Math.abs(deltaY) < 0.05 && Math.abs(deltaRad) > 0) encYst = EncoderStatus.SmallDelta;
        else encYst = EncoderStatus.InMoving;

        if(deltaRad == 0) encRadSt = EncoderStatus.ZeroDelta;
        else if(Math.abs(deltaRad) == 0 ||  Math.abs(deltaRad) < 0.05) encRadSt = EncoderStatus.SmallDelta;
        else encRadSt = EncoderStatus.InMoving;

        deltaPosition.setHeading(deltaRad);
        deltaPosition.setX(deltaX);
        deltaPosition.setY(deltaY);

        // Векторный поворот и добавление глобального перемещения к глобальным координатам
        globalPosition.add(Vector2.rotate(deltaPosition.toVector(), globalPosition.getHeading()) , deltaPosition.getHeading());

        globalPosition.setHeading(globalPosition.getHeading());
    }

    // Вывод позиции робота
    public synchronized void getRobotPos(){
        op.telemetry.addLine("Robot position")
                .addData("\nX:", globalPosition.getX())
                .addData("\nY", globalPosition.getY())
                .addData("\nHeading", globalPosition.getHeading() * 57.29);
        op.telemetry.addLine();
    }
    // Вывод позиции энкодеров
    public synchronized void getEncPos(){
        op.telemetry.addLine("Encoders statements")
                .addData("\nEncL", encL.getCurrentPosition())
                .addData("\nEncM", encM.getCurrentPosition())
                .addData("\nEncR", encR.getCurrentPosition());
        op.telemetry.addLine();
    }
}
