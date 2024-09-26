package org.firstinspires.ftc.teamcode;


public interface CONSTS {//ВСЁ В СМ!!!!
    double RPM_MOTOR_WHEEL = 435;
    double RPS_MOTOR_WHEEL = 435.0 / 60.0;
    double TPR_WHEEL = 385;
    double WIDTH_WHEEL = 9.6;
    double TPR_ENCODER = 2000;
    double TPS_WHEEL = (RPM_MOTOR_WHEEL/60) * TPR_WHEEL;
    double WIDTH_ENC_WHEEL = 4.8;
    double LENGHT_ENC_WHEEL = WIDTH_ENC_WHEEL * Math.PI;
    double TICK_PER_CM = TPR_ENCODER/ (LENGHT_ENC_WHEEL);
    double TICK_PER_DEGR = TPR_ENCODER/360;
    double DIAM_CIRCLE_ROBOT = 42.7;
    double DIST_BETWEEN_WHEEL_Y = 30.2;
    double DIST_BETWEEN_WHEEL_X = 30.16;
    double OFFSET_ENC_M_FROM_CENTER = 1.53;
    double DIST_TO_ENC_R_FROM_ENC_M = 15.25;
    double DIST_TO_ENC_L_FROM_ENC_M = 15.25;
    double DIST_BETWEEN_ENC_X = 30.5;
    double LENGHT_ROUND_BIG = 42.7 * Math.PI;
    double LENGHT_ROUND_SMALL = 30.5 * Math.PI;
    double DIST_FIELD_Y = 302.26;
    double DIST_FIELD_X = 302.26;
    double RATIO_WHEEL_ENCODER = WIDTH_WHEEL / WIDTH_ENC_WHEEL;
    double RPS_ENC_WHEEL = RPS_MOTOR_WHEEL * RATIO_WHEEL_ENCODER;
    double MAX_TPS_ENCODER = TPR_ENCODER * RPS_ENC_WHEEL;

    double CLOSE = 0.0;
    double OPEN = 0.5;
}

