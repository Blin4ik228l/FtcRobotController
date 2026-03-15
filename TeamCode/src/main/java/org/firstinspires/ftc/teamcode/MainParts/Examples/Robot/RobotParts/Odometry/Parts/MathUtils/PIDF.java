package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Module;

public class PIDF extends Module {
    private ElapsedTime runtime = new ElapsedTime();
    protected double kP, kI, kD, kF; // Пропорциональный, интегральный, дифференциальный коэффициент, feedForward

    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double F = 0;
    private final double maxI;
    private final double minI;

    private double error, olderror, oldtime;
    private double target, current;

    private String pidName;
    public PIDF(double kP, double kI, double kD, double kF, double minI , double maxI, String pidName){
        this.kP = kP;   // Максимально приблизить к результату, но не больше результат(Борис Бритва)
        this.kI = kI;   // Добивает до нужного результата(Борис Бритва добить)
        this.kD = kD;   // Сглаживает колебания
        this.kF = kF;
        this.minI = minI;
        this.maxI = maxI;
        this.pidName = pidName;
    }

    public void setPID(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public double getkF() {
        return kF;
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
        this.target = target;
        this.current = current;

        error = target - current;

        P = error * kP;
        I += error * (runtime.milliseconds() - oldtime) * kI;
        D = (error - olderror) /(runtime.milliseconds() - oldtime) * kD;
        F = target * kF;

        olderror = error;
        oldtime = runtime.milliseconds();

        return P + Range.clip(I, minI, maxI) + D + F;
    }

    public double calculate(double error){
        P = error * kP;
        I += error * (runtime.milliseconds() - oldtime) * kI;
        D = (error - olderror) /(runtime.milliseconds() - oldtime) * kD;

        olderror = error;
        oldtime = runtime.milliseconds();

        return P + Range.clip(I, minI, maxI) + D;
    }

    @Override
    public void sayModuleName() {
        telemetry.addLine(pidName);
    }
    @Override
    protected void showDataExt() {
        telemetry.addLine(String.format("P %.3f I %.3f D %.3f F %.3f", P, I, D, F));
        telemetry.addLine(String.format("kP %s kI %s kD %s kF %s", kP, kI, kD, kF));
        telemetry.addLine(String.format("Target %.3f Current %.3f", target, current));
        if (target != 0){
            telemetry.addData("error", ((target - current) / target) * 100);
        }else telemetry.addData("error", 0);
    }
    @Override
    protected void sayLastWords() {

    }
}
