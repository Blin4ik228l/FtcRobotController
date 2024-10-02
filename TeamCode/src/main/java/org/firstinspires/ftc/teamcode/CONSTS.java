package org.firstinspires.ftc.teamcode;


public interface CONSTS {//ВСЁ В СМ!!!!
    //просто значения(см)
    double WIDTH_ENC_WHEEL = 4.8;// ширина энкодера
    double WIDTH_WHEEL = 9.6;// ширина колеса
    double DIST_BETWEEN_WHEEL_Y = 30.2;
    double DIST_BETWEEN_WHEEL_X = 30.16;
    double DIST_TO_ENC_R_FROM_ENC_M = 15.25;
    double DIST_TO_ENC_L_FROM_ENC_M = 15.25;
    double OFFSET_ENC_M_FROM_CENTER = 1.53;//сдвиг линий энкодеров от середины робота
    double DIAM_CIRCLE_ROBOT = 42.7;// диаметр большей окружности робота(колёс)
    double DIST_BETWEEN_ENC_X = 30.5;
    double DIST_FIELD_Y = 302.26;//длина поля
    double DIST_FIELD_X = 302.26;// ширина поля

    //ДЛИНЫ ОКРУЖНОСТЕЙ(см)
    double LENGTH_ROUND_BIG = 42.7 * Math.PI;// длина большей окружности
    double LENGTH_ROUND_SMALL = 30.5 * Math.PI;// длина меньшей окружности
    double LENGTH_ENC_WHEEL = WIDTH_ENC_WHEEL * Math.PI; //длина окружности колеса энкодера
    double LENGTH_ROUND_WHEEL = WIDTH_WHEEL * Math.PI;

    //ОТНОШЕНИЕ КОЛЁС К ШЕСТЕРЁНКАМ ИЛИ ЭНКОДЕРУ
    double RATIO_WHEEL_ENCODER = WIDTH_WHEEL / WIDTH_ENC_WHEEL;//отношение колеса к колесу энкодера

    //СКОРОСТЬ МОТОРОВ ИЛИ КОЛЕСА ЭНКОДЕРА(обор(см)/мин(сек))
    double RPM_MOTOR_WHEEL = 435; // обор/мин
    double RPS_MOTOR_WHEEL = 435.0 / 60.0;// обор/сек
    double RPS_ENC_WHEEL = RPS_MOTOR_WHEEL * RATIO_WHEEL_ENCODER;// обор/сек энкодера
    double CMPS_MOTOR_WHEEL = RPS_MOTOR_WHEEL * LENGTH_ROUND_WHEEL;// см/сек колеса
    double CMPS_ENC_WHEEL = RPS_ENC_WHEEL * LENGTH_ENC_WHEEL;// см/сек энкодер

    //СКОРОСТЬ МОТОРОВ ИЛИ КОЛЕСА ЭНКОДЕРА (тики/обор)
    double TPR_WHEEL = 385;// тик/обор
    double TPR_ENCODER = 2000;// тики/обор

    //СКОРОСТЬ МОТОРОВ ИЛИ КОЛЕСА ЭНКОДЕРА В ТИКАХ(тики/мин(сек)(см)(градус))
    double TPS_WHEEL = (RPM_MOTOR_WHEEL/60) * TPR_WHEEL;// тики/сек колеса
    double TICK_PER_CM = TPR_ENCODER/ (LENGTH_ENC_WHEEL); // тик/см энкодера
    double TICK_PER_DEGREES = TPR_ENCODER/360;  // тик/градус энкодера
    double MAX_TPS_ENCODER = TPR_ENCODER * RPS_ENC_WHEEL;//максимальная скорость тик/сек энкодера

    //ЗНАЧЕНИЕ ДЛЯ СЕРВАКОВ
    double CLOSE = 0.0;
    double OPEN = 0.5;
}

