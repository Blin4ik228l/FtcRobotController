package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// Отдельный класс, работающий с одометрией в отдельном потоке
class Odometry extends Thread{
    Odometry (Position startPosition, DcMotorEx encL, DcMotorEx encR, DcMotorEx encM){
        this.globalPosition = new Position(startPosition);
        this.deltaPosition = new Position();

        this.encL = encL;
        this.encR = encR;
        this.encM = encM;

        this.velocity = new Matrix(3,1);
        this.oldVelocity = new Matrix(3,1);
        this.acceleration = new Matrix(3,1);

        this.velocity.Mat = new double[][]{{0,0,0}};
        this.oldVelocity.Mat = new double[][]{{0,0,0}};
        this.acceleration.Mat = new double[][]{{0,0,0}};

        runtime = new ElapsedTime();
        oldTime = 0;
        dt = 0;
    }

    private final ElapsedTime runtime;                                // Пройденное время
    private double oldTime;                                           // Предыдущее время
    private double dt;                                                // Разница во времени
    private double encLOld, encROld, encMOld;                         // Значения энкодера на предыдущем шаге
    private final DcMotorEx encM, encL, encR;                         // Объекты энкодеров
    private final Position deltaPosition, globalPosition;             // Относительное перемещение и глобальное положение
    private final Matrix velocity, oldVelocity, acceleration;         // Вектора скорость и ускорение ОТНОСИТЕЛЬНО КООРДИНАТ РОБОТА

    @Override
    // Запускаем методы для обновления данных, пока объект существует
    public void run() {
        while (this.isAlive()){
            dt = (runtime.milliseconds() - oldTime) / 1000;
            updateGlobalPosition();
            updateAcceleration();
            updateVelocity();
            oldTime = runtime.milliseconds();
        }
    }
    // Тики энкодера в сантиметры
    private double ticksToCm(double ticks){
        return ticks / CONSTS.TICK_PER_CM;
    }

    // Геттер глобального положения робота
    public synchronized Position getGlobalPosition(){
        return globalPosition;
    }

    // Геттер вектора скорости
    public synchronized Matrix getVelocity(){
        return new Matrix(velocity);
    }

    // Геттер вектора ускорения
    public synchronized Matrix getAcceleration(){
        return new Matrix(acceleration);
    }

    // Обновление вектора скорости робота
    private void updateVelocity(){
        velocity.Mat = new double [][] {{   deltaPosition.getX() / dt,
                                            deltaPosition.getY() / dt,
                                            0   }};
        oldVelocity.Mat = velocity.Mat;
        oldTime = runtime.milliseconds();
    }

    // Обновление вектора ускорения робота
    private void updateAcceleration(){
        for (int i = 0; i < 3; i++)
            acceleration.Mat[i][0] = (velocity.Mat[0][0] - oldVelocity.Mat[i][0]) / dt;
    }

    // Обновление положения робота на поле с помощью следящих колес
    private void updateGlobalPosition(){
        double leftEncoderXNow = ticksToCm(encL.getCurrentPosition());
        double deltaLeftEncoderX = leftEncoderXNow - encLOld;
        encLOld = leftEncoderXNow;

        double rightEncoderXnow = ticksToCm(encR.getCurrentPosition());
        double deltaRightEncoderX = rightEncoderXnow - encROld;
        encROld = rightEncoderXnow;

        double encoderYnow = ticksToCm(encM.getCurrentPosition());
        double deltaEncoderY = encoderYnow - encMOld;
        encMOld = encoderYnow;

        // Если перемещения не было - выходим из метода
        if(leftEncoderXNow == 0 && rightEncoderXnow == 0 && encoderYnow == 0 ) {
            return;
        }

        // Расчет перемещений робота за время, пройденное с момента предыдущего вызова метода
        // Для корректной работы этот метод должен работать в непрерывном цикле
        double deltaRad = (deltaRightEncoderX + deltaLeftEncoderX)/CONSTS.DIST_BETWEEN_WHEEL_X;
        double deltaX = ticksToCm(deltaLeftEncoderX + deltaRightEncoderX) / 2.0;
        double deltaY = ticksToCm(deltaEncoderY) - deltaRad * CONSTS.OFFSET_ENC_M_FROM_CENTER;

        deltaPosition.setHeading(deltaRad);
        deltaPosition.setX(deltaX);
        deltaPosition.setY(deltaY);

        // Матричный поворот и добавление глобального перемещения к глобальным координатам
        globalPosition.increment(deltaPosition.turn(globalPosition.getHeading()));

        // Если направление робота будет больше +-2pi радиан (+-360 градусов), то приравняется
        // к остатку от деления на 2pi (360)
        if (Math.abs(globalPosition.getHeading()) >= 2 * Math.PI) {
            globalPosition.setHeading(globalPosition.getHeading() % 2 * Math.PI);
        }
    }

}
