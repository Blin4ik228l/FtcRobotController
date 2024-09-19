package org.firstinspires.ftc.teamcode;


public interface CONSTS {
    double RPM_MOTOR_WHEEL = 300;
    double TPR_WHEEL = 560;
    double TPR_ENCODER = 2000;
    double TPS_WHEEL = (RPM_MOTOR_WHEEL/60) * TPR_WHEEL;
    double WIDTH_ENC_WHEEL = 48;
    double LENGHT_ENC_WHEEL = WIDTH_ENC_WHEEL * 3.14;
    double TICK_PER_CM = TPR_ENCODER/ (LENGHT_ENC_WHEEL);
    double DIAM_CIRCLE_ROBOT = 427;
    double DIST_BETWEEN_WHEEL_Y = 302;
    double DIST_BETWEEN_WHEEL_X = 301.6;
    double OFFSET_ENC_M_FROM_CENTER = 15.3;
    double DIST_TO_ENC_R_FROM_ENC_M = 152.5;
    double DIST_TO_ENC_L_FROM_ENC_M = 152.5;
    double DIST_BETWEEN_ENC_X = 305;
    double LENGHT_ROUND_BIG = 427 * 31;
    double LENGHT_ROUND_SMALL = 305 * 3.14;

    double CLOSE = 0.6;
    double OPEN = 0.9;
}

