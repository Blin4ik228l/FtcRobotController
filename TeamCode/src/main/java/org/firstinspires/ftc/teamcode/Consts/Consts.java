package org.firstinspires.ftc.teamcode.Consts;


public interface Consts { // ВСЁ В СМ!!!!
    double RAZN_ENCODERS = 263254.0/250005.0;

    // просто значения(см)
    double DIAM_ENC_WHEEL = 4.8;                                            // диаметр колеса энкодера
    double DIAM_WHEEL = 9.6;                                                // диаметр колеса
    double DIST_BETWEEN_WHEEL_Y = 30.2;                                     //
    double DIST_BETWEEN_WHEEL_X = 30.16;                                    // расстояние между колесами
    double DIST_TO_ENC_R_FROM_ENC_M = 15.25;                                //
    double DIST_TO_ENC_L_FROM_ENC_M = 15.25;                                //
    double OFFSET_ENC_M_FROM_CENTER = 0.53;                                 // сдвиг линий энкодеров от середины робота
    double DIAM_CIRCLE_ROBOT = 42.7;                                        // диаметр большей окружности робота(колёс)
    double DIST_BETWEEN_ENC_X = 30.8775;                                       // расстояние между энкодерами R и L     // ширина поля
    double MAX_DIST = 366;                                               //
    double MAX_RAD = Consts.LENGTH_ROUND_BIG/(Consts.DIAM_CIRCLE_ROBOT/2);  //
    double MAX_TIME_DRIVE = MAX_DIST/ Consts.MAX_CM_PER_SEC;
    double MAX_TIME_ROUNDING = MAX_RAD/ Consts.MAX_RAD_PER_SEC;

    // ДЛИНЫ ОКРУЖНОСТЕЙ(см)
    double LENGTH_ROUND_BIG = DIAM_CIRCLE_ROBOT * Math.PI;                  // длина большей окружности
    double LENGTH_ROUND_SMALL = 30.5 * Math.PI;                             // длина меньшей окружности
    double LENGTH_ENC_WHEEL = DIAM_ENC_WHEEL * Math.PI;                     // длина окружности колеса энкодера
    double LENGTH_ROUND_WHEEL = DIAM_WHEEL * Math.PI;                       // длина окружности колеса
    double ROUND_BARABAN = 11.6;

    // ОТНОШЕНИЕ КОЛЁС К ШЕСТЕРЁНКАМ ИЛИ ЭНКОДЕРУ
    double RATIO_WHEEL_ENCODER = DIAM_WHEEL / DIAM_ENC_WHEEL;               // отношение колеса к колесу энкодера

    // СКОРОСТЬ МОТОРОВ ИЛИ КОЛЕСА ЭНКОДЕРА ( обор(см)/мин(сек) )
    double RPM_MOTOR_WHEEL = 435;                                           // скорость колеса обор/мин
    double RPS_MOTOR_WHEEL = 435.0 / 60.0;                                  // обор/сек
    double RPS_ENC_WHEEL = RPS_MOTOR_WHEEL * RATIO_WHEEL_ENCODER;           // обор/сек энкодера
    double CMPS_MOTOR_WHEEL = RPS_MOTOR_WHEEL * LENGTH_ROUND_WHEEL;         // см/сек колеса
    double CMPS_ENC_WHEEL = RPS_ENC_WHEEL * LENGTH_ENC_WHEEL;               // см/сек энкодер

    // СКОРОСТЬ МОТОРОВ ИЛИ КОЛЕСА ЭНКОДЕРА (тики/обор)
    double TPR_WHEEL = 577.7;                                               // тик/обор
    double TPR_ENCODER = 2000;                                              // тики/обор

    // СКОРОСТЬ МОТОРОВ ИЛИ КОЛЕСА ЭНКОДЕРА В ТИКАХ(тики/мин(сек)(см)(градус))
    double TICK_PER_CM_WHEEL = TPR_WHEEL/LENGTH_ROUND_WHEEL;
    double TPS_WHEEL = (RPM_MOTOR_WHEEL/60) * TPR_WHEEL;                    // тики/сек колеса
    double TICK_PER_CM = TPR_ENCODER/ (LENGTH_ENC_WHEEL);                   // тик/см энкодера
    double TICK_PER_DEGREES = TPR_ENCODER/360;                              // тик/градус энкодера
    double MAX_TPS_ENCODER = TPR_ENCODER * RPS_ENC_WHEEL;                   // максимальная скорость тик/сек энкодера
    double MAX_RAD_PER_SEC = CMPS_ENC_WHEEL/(DIST_BETWEEN_ENC_X/2);         //
    double MAX_CM_PER_SEC = RPS_ENC_WHEEL * LENGTH_ENC_WHEEL;
    double TICK_PER_CM_BARABAN = TPR_WHEEL/ROUND_BARABAN;

    // СКОРОСТИ И УСКОРЕНИЯ РОБОТА
    double MAX_LINEAR_SPEED = 220;
    double MAX_LINEAR_FORWARD = 220;
    double MAX_LINEAR_SIDE = 190;
    double MAX_ANGULAR_SPEED = 6.28;
    double MAX_LINEAR_ACCEL = 1000;
    double MAX_ANGULAR_ACCEL = 60;
    double MID_LINEAR_SPEED = 40;
    double MIN_LINEAR_SPEED = 15;    //25
    double MIN_ANGULAR_SPEED = 2;
    double MIN_LINEAR_ACCEL = 1;
    double MIN_ANGULAR_ACCEL = 1;

    // COLOR SENSER
    double KpRed = 1.76;
    double KpGreen = 1.001;
    double KpBlue = 1.08;
}

