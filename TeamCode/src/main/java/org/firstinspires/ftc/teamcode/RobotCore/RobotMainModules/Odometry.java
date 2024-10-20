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
    private double dt;                                                          // Разница во времени
    private double encLOld, encROld, encMOld;                                   // Значения энкодера на предыдущем шаге
    private double angularVelocity, angularAcceleration, oldAngularVelocity;
    public DcMotorEx encM;                                                // Объекты энкодеров
    public DcMotorEx encL;
    public DcMotorEx encR;
    private final Position deltaPosition;
    private final Position globalPosition;                                      // Относительное перемещение и глобальное положение
    private final Vector2 velocity, oldVelocity, acceleration;                  // Вектора скорость и ускорение ОТНОСИТЕЛЬНО КООРДИНАТ РОБОТА

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
        this.encR =  op.hardwareMap.get(DcMotorEx.class, "rightB");

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
            dt = (runtime.milliseconds() - oldTime) / 1000;
            updateGlobalPosition();
            updateAcceleration();
            updateVelocity();
            updateAngularAcceleration();
            updateAngularVelocity();
            oldTime = runtime.milliseconds();
        }
    }
    // Тики энкодера в сантиметры
    public double ticksToCm(double ticks){
        return ticks / CONSTS.TICK_PER_CM;
    }

    public synchronized  void setGlobalPosition(Position position) {
        globalPosition.x = position.x;
        globalPosition.y = position.y;
        globalPosition.heading = position.heading;
    }

    // Геттер глобального положения робота
    public synchronized Position getGlobalPosition(){
        return globalPosition;
    }

    // Геттер вектора скорости
    public synchronized Vector2 getVelocity(){
        return new Vector2(velocity);
    }

    // Геттер вектора ускорения
    public synchronized Vector2 getAcceleration(){
        return new Vector2(acceleration);
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
        return velocity.mag();
    }
    // Обновление вектора скорости робота
    private synchronized void updateVelocity(){
        oldVelocity.x = velocity.x;
        oldVelocity.y = velocity.y;

        velocity.x = deltaPosition.toVector().x;
        velocity.y = deltaPosition.toVector().y;

        velocity.divide(dt);
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
    private void updateGlobalPosition(){
        double leftEncoderXNow = encL.getCurrentPosition();
        double deltaLeftEncoderX = leftEncoderXNow - encLOld;
        encLOld = leftEncoderXNow;

        double rightEncoderXnow = encR.getCurrentPosition();
        double deltaRightEncoderX = rightEncoderXnow - encROld;
        encROld = rightEncoderXnow;

        double encoderYnow = encM.getCurrentPosition();
        double deltaEncoderY = encoderYnow - encMOld;
        encMOld = encoderYnow;

        // Если перемещения не было - выходим из метода
        if(leftEncoderXNow == 0 && rightEncoderXnow == 0 && encoderYnow == 0 ) {
            return;
        }

        // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
        // Для корректной работы этот метод должен работать в непрерывном цикле
        double deltaRad = ticksToCm(deltaRightEncoderX + deltaLeftEncoderX)/CONSTS.DIST_BETWEEN_WHEEL_X;
        double deltaX = ticksToCm(deltaLeftEncoderX + deltaRightEncoderX) / 2.0;
        double deltaY = ticksToCm(deltaEncoderY) - deltaRad * CONSTS.OFFSET_ENC_M_FROM_CENTER;

        deltaPosition.setHeading(deltaRad);
        deltaPosition.setX(deltaX);
        deltaPosition.setY(deltaY);

        // Матричный поворот и добавление глобального перемещения к глобальным координатам
        globalPosition.add(Vector2.rotate(deltaPosition.toVector(), globalPosition.heading), deltaPosition.heading );

        // Если направление робота будет больше +-2pi радиан (+-360 градусов), то приравняется
        // к остатку от деления на 2pi (360)
        if (Math.abs(globalPosition.getHeading()) >= 2 * Math.PI) {
            globalPosition.setHeading(globalPosition.getHeading() % 2 * Math.PI);
        }
    }

}
