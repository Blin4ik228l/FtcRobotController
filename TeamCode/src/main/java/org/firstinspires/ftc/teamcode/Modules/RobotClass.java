package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.TeleSkope;

public class RobotClass {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */

    public RobotClass(OpMode op){
        driveTrain = new MecanumDriveTrain(op);
        teleSkope = new TeleSkope(op);
    }
    public MecanumDriveTrain driveTrain;
    public TeleSkope teleSkope;
    public void start(){

    }
}
