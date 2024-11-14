package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

// Отдельный класс, работающий с одометрией в отдельном потоке
public class Odometry extends Thread implements Module {
    private final OpMode op;

    private final ElapsedTime runtime;                                          // Пройденное время
    private double oldTime;                                                     // Предыдущее время
     double dt;                                                          // Разница во времени
    private double encLOld, encROld, encMOld;                                   // Значения энкодера на предыдущем шаге
    private double angularVelocity, angularAcceleration, oldAngularVelocity;
    public  DcMotorEx encM;                                                // Объекты энкодеров
    public volatile DcMotorEx encL;
    public volatile DcMotorEx encR;
    private final Position deltaPosition;
    private final Position globalPosition;                                      // Относительное перемещение и глобальное положение
    private final Vector2 velocity, oldVelocity, acceleration;                  // Вектора скорость и ускорение ОТНОСИТЕЛЬНО КООРДИНАТ РОБОТА
    private double maxAcceleration, maxVel;

    public  Odometry (Position startPosition, OpMode op){
        this.op = op;

        this.globalPosition = new Position(startPosition);
        this.deltaPosition = new Position();

        this.velocity = new Vector2(0,0);
        this.oldVelocity = new Vector2(0,0);
        this.acceleration = new Vector2(0,0);

        runtime = new ElapsedTime();
        oldTime = 0;
        dt = 0;
    }

    public  Odometry (OpMode op){
        this(new Position(0,0,0), op);
    }

    @Override
    public void init() {
        this.encM = op.hardwareMap.get(DcMotorEx.class, "encM") ;
        this.encL = op.hardwareMap.get(DcMotorEx.class, "leftB") ;
        this.encR = op.hardwareMap.get(DcMotorEx.class, "rightB");

        encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.setDaemon(true);
        this.start();
    }

    @Override
    // Запускаем методы для обновления данных, пока объект существует
    public void run() {
        while (this.isAlive()){
            dt = (runtime.milliseconds() - oldTime)/1000.0;
            oldTime = runtime.milliseconds();
            updateGlobalPosition();
            updateVelocity();
            updateMaxVel();
            updateAcceleration();
            updateMaxAcceleration();
            updateAngularAcceleration();
            updateAngularVelocity();
        }
    }
    // Тики энкодера в сантиметры
    public synchronized double ticksToCm(double ticks){
        return ticks / CONSTS.TICK_PER_CM;
    }

    public synchronized void setGlobalPosition(Position position) {
        globalPosition.x = position.x;
        globalPosition.y = position.y;
        globalPosition.heading = position.heading;
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

    public synchronized double getMaxVel(){
        return maxVel;
    }

    public synchronized double getMaxAcceleration(){
       return maxAcceleration;
    }



    // Геттер глобального положения робота
    public synchronized Position getGlobalPosition(){
        return globalPosition;
    }

    // Геттер вектора скорости
    public synchronized Vector2 getVelocity(){
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
    // Обновление вектора скорости робота
    private synchronized void updateVelocity(){
        oldVelocity.x = velocity.x;
        oldVelocity.y = velocity.y;

        velocity.x = (deltaPosition.toVector().x)/dt;
        velocity.y = (deltaPosition.toVector().y)/dt;

    }

    // Обновление вектора ускорения робота
    private synchronized void updateAcceleration() {
        acceleration.x = (velocity.x - oldVelocity.x)/dt;
        acceleration.y = (velocity.y - oldVelocity.y)/dt;
    }

    private synchronized void updateAngularVelocity(){
        oldAngularVelocity = angularVelocity;
        angularVelocity = deltaPosition.heading/dt;
    }

    // Обновление вектора ускорения робота
    private synchronized void updateAngularAcceleration() {
        angularAcceleration = (angularAcceleration - oldAngularVelocity)/dt;
    }

    // Обновление положения робота на поле с помощью следящих колес
    private synchronized void updateGlobalPosition(){
        double leftEncoderXNow = ticksToCm(encL.getCurrentPosition());
        double deltaLeftEncoderX = leftEncoderXNow - encLOld;
        encLOld = leftEncoderXNow;

        double rightEncoderXNow = ticksToCm(encR.getCurrentPosition());
        double deltaRightEncoderX = rightEncoderXNow - encROld;
        encROld = rightEncoderXNow;

        double encoderYNow = ticksToCm(encM.getCurrentPosition());
        double deltaEncoderY = encoderYNow - encMOld;
        encMOld = encoderYNow;

        // Если перемещения не было - выходим из метода
        if(deltaLeftEncoderX == 0 && deltaRightEncoderX == 0 && deltaEncoderY == 0 ) {
            return;
        }

        // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
        // Для корректной работы этот метод должен работать в непрерывном цикле
        double deltaRad = (deltaRightEncoderX + deltaLeftEncoderX)/CONSTS.DIST_BETWEEN_WHEEL_X;
        double deltaX = (deltaLeftEncoderX - deltaRightEncoderX )/ 2.0;
        double deltaY = (deltaEncoderY) - deltaRad * CONSTS.OFFSET_ENC_M_FROM_CENTER;

        deltaPosition.setHeading(deltaRad);
        deltaPosition.setX(deltaX);
        deltaPosition.setY(deltaY);

        // Векторный поворот и добавление глобального перемещения к глобальным координатам
        globalPosition.add(Vector2.rotate(deltaPosition.toVector(), globalPosition.heading), deltaPosition.heading );

        // Если направление робота будет больше +-2pi радиан (+-360 градусов), то приравняется
        // к остатку от деления на 2pi (360)
        if (Math.abs(globalPosition.getHeading()) >= 2 * Math.PI) {
            globalPosition.setHeading(globalPosition.getHeading() % 2 * Math.PI);
        }
    }

}
