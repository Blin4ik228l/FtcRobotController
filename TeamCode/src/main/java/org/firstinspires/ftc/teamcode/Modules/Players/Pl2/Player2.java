package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Players.Player;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class Player2 extends Player implements Runnable {
   public Player2(Gamepad gamepad, RobotClass.Collector teleSkope, OpMode op){
       super(op.telemetry);

        playersGamepad = gamepad;
        this.collector = teleSkope;
        joystickActivity = new JoystickActivity();
    }
    public RobotClass.Collector collector;
    public JoystickActivity joystickActivity;

    @Override
    public void run() {

    }

    @Override
    public void play() {
        joystickActivity.checkActivity();

//        double inTakePower = 0;
//        double flyWheelVelocity = 0;
//        double barabanPos = BARABAN_START_POS;
//        double pusherPos = PUSHER_START_POS;
//        double anglePos = ANGLE_START_POS;
//
//        if(joystickActivity.buttonA){
////            onFlyWheel = true;
//            flyWheelVelocity = 5;
//        }
//
//        collector.setPowerAndPos(inTakePower, flyWheelVelocity, barabanPos, pusherPos, anglePos);

        showData();
    }

    @Override
    public void showData() {
        collector.colorSensor.showData();
        collector.servos.showData();
        collector.encoders.showData();
    }
}
