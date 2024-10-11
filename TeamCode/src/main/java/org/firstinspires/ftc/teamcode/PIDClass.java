package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDClass {
    ElapsedTime runtime = new ElapsedTime();
    double kP, KI, kD, error, oldError, integral;
    PIDClass(double kP, double KI, double kD, double error, double oldError, double integral){

    }
}
