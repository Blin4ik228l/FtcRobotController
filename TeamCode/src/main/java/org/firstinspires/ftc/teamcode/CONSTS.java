package org.firstinspires.ftc.teamcode;


public interface CONSTS {//ВСЁ В СМ!!!!
    double RPM_MOTOR_WHEEL = 300;
    double TPR_WHEEL = 560;
    double TPR_ENCODER = 2000;
    double TPS_WHEEL = (RPM_MOTOR_WHEEL/60) * TPR_WHEEL;
    double WIDTH_ENC_WHEEL = 4.8;
    double LENGHT_ENC_WHEEL = WIDTH_ENC_WHEEL * 3.14;
    double TICK_PER_CM = TPR_ENCODER/ (LENGHT_ENC_WHEEL);
    double DIAM_CIRCLE_ROBOT = 42.7;
    double DIST_BETWEEN_WHEEL_Y = 30.2;
    double DIST_BETWEEN_WHEEL_X = 30.16;
    double OFFSET_ENC_M_FROM_CENTER = 1.53;
    double DIST_TO_ENC_R_FROM_ENC_M = 15.25;
    double DIST_TO_ENC_L_FROM_ENC_M = 15.25;
    double DIST_BETWEEN_ENC_X = 30.5;
    double LENGHT_ROUND_BIG = 42.7 * 3.14;
    double LENGHT_ROUND_SMALL = 30.5 * 3.14;
    double DIST_FIELD_Y = 302.26;
    double DIST_FIELD_X = 302.26;

    double CLOSE = 0.0;
    double OPEN = 0.5;
}

