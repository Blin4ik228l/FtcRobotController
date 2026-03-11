package org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers;

public interface Theory {
    /**
    *Важное примечание я работаю в системе (сантиметры / секунда)
    * Все значения я пишу в см
    */

    //Максимальные теоретические скорости(линейная и угловая)

    //Робот
    double DRIVETRAIN_MOTOR_RPM = 435;
    double DRIVETRAIN_WHEEL_RADIUS = 4.8;

    //ЕСЛИ энкодер это отдельное следящее колесо указываем его радиус, если энкодер встроен в мотор то указывайте радиус колеса на нём
    double DRIVETRAIN_ENCODER_RADIUS = 2.4;
    double DRIVETRAIN_GEAR_RATIO = DRIVETRAIN_WHEEL_RADIUS / DRIVETRAIN_ENCODER_RADIUS;
    double DRIVETRAIN_ENC_OUTPUT_MAX_SPEED = DRIVETRAIN_MOTOR_RPM * DRIVETRAIN_GEAR_RATIO;
    double DRIVETRAIN_DISTANCE_BETWEEN_ENCODERS = 30.0;

    double MAX_ROBOT_LINEAR_SP = (DRIVETRAIN_ENC_OUTPUT_MAX_SPEED * 2.0 * Math.PI * DRIVETRAIN_ENCODER_RADIUS) / 60.0;
    double MAX_ROBOT_HEAD_SP = MAX_ROBOT_LINEAR_SP / (DRIVETRAIN_DISTANCE_BETWEEN_ENCODERS / 2.0);

    //Турель
    double TURRET_MOTOR_RPM = 435;

    double TURRET_WHEEL_RADIUS = 10.38;
    double TURRET_ENCODER_RADIUS = 2;

    double TURRET_GEAR_RATIO = TURRET_WHEEL_RADIUS / TURRET_ENCODER_RADIUS;
    double TURRET_ENC_OUTPUT_MAX_SPEED = TURRET_MOTOR_RPM * TURRET_GEAR_RATIO;

    double MAX_TURRET_LINEAR_SP = (TURRET_ENC_OUTPUT_MAX_SPEED * 2.0 * Math.PI * TURRET_ENCODER_RADIUS) / 60.0;
    double MAX_TURRET_HEAD_SP = (MAX_TURRET_LINEAR_SP / TURRET_ENCODER_RADIUS) / TURRET_GEAR_RATIO;
}
