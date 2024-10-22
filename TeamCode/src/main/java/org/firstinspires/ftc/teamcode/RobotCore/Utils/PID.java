package org.firstinspires.ftc.teamcode.RobotCore.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

// В этом классе описывается ПИД регулятор.

public class PID {
    private ElapsedTime runtime = new ElapsedTime();
    private double kP, kI, kD; // Пропорциональный, интегральный, дифференциальный коэффициент

    public double P = 0;
    public double I = 0;
    public double D = 0;

    public double error, olderror, oldtime;

    public PID (double kP, double kI, double kD){
        this.kP = kP; // Максимально приблизить к результату, но не больше результат(Борис Бритва)
        this.kI = kI; // Добивает до нужного результата(Борис Бритва добить)
        this.kD = kD; // Сглаживает колебания
    }

    public void setPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset(){
        P = 0;
        I = 0;
        D = 0;
        olderror = 0;
        oldtime = 0;

        runtime.reset();
    }

    public double calculate(double target, double current){
        error = target - current;

        P = error * kP;
//        I += error * (runtime.milliseconds() - oldtime) * kI;
//        D = (error - olderror) /(runtime.milliseconds() - oldtime) * kD;

        olderror = error;
        oldtime = runtime.milliseconds();

        return P + I + D;
    }
}