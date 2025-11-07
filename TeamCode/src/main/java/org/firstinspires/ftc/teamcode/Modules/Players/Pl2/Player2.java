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
        while (true){
            play();
        }
    }

    @Override
    public void play() {
        joystickActivity.checkActivity();

        boolean onInTake = false;
        boolean onFlyWheel = false;
        double flyWheelVelocity = 0;

        if(joystickActivity.buttonA){
            onInTake = true;
//            onFlyWheel = true;
            flyWheelVelocity = 628;
        }

        collector.setAll(onInTake, onFlyWheel, flyWheelVelocity);

        showData();
    }

    @Override
    public void showData() {
        collector.automaticClass.showCells();
        collector.encoders.showFLyWheelSpeed();
        collector.colorSensor.showDominantColor();
        collector.servos.showServosPos();
        collector.automaticClass.showCount();
//        collector.colorSensor.showData();
    }
}
