package org.firstinspires.ftc.teamcode;


public interface CONSTS {
    double RPM_MOTOR_WHEEL = 300;
    double TPR_WHEEL = 560;
    double TPR_ENCODER = 2000;
    double TPS_WHEEL = (RPM_MOTOR_WHEEL/60) * TPR_WHEEL;
    double WIDTH_ENC_WHEEL = 4.8;
    double LENGHT_ENC_WHEEL = WIDTH_ENC_WHEEL * 3.14;
    double TICK_PER_CM = TPR_ENCODER/ (LENGHT_ENC_WHEEL);

    double DIST_TO_ENC_Y_FROM_CENTER = 1;
    double DIST_TO_ENC_X_FROM_CENTER = 1;

    double CLOSE = 0.6;
    double OPEN = 0.9;
}

