package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// В этом классе описывается ПИД регулятор.

public class PID {
    private ElapsedTime runtime = new ElapsedTime();
    private double kP, kI, kD; // Пропорциональный, интегральный, дифференциальный коэффициент

    private double P = 0;
    private double I = 0;
    private double D = 0;

    private double error, olderror, oldtime;

    PID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    void setPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    void reset(){
        P = 0;
        I = 0;
        D = 0;
        olderror = 0;
        oldtime = 0;

        runtime.reset();
    }

    double calculate(double target, double current){
        // TODO:
        // Расписать алгоритм работы ПИД регулятора

        error = target - current;

        P = 1;
        I += 1;
        D = 1;

        olderror = error;
        oldtime = runtime.milliseconds();

        return P + I + D;
    }
}