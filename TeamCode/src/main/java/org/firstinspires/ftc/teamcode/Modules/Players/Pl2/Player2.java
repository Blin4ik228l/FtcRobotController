package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Players.Player;
import org.firstinspires.ftc.teamcode.Robot.AutomaticClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class Player2 extends Player implements Runnable {
   public Player2(Gamepad gamepad, RobotClass.Collector teleskope, OpMode op){
       super(op.telemetry);

        playersGamepad = gamepad;
        this.collector = teleskope;
        joystickActivity = new JoystickActivity();
    }
    public RobotClass.Collector collector;
    public JoystickActivity joystickActivity;
    public AutomaticClass automaticClass;

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()){
            play();
        }
    }

    @Override
    public void play() {
        joystickActivity.checkActivity();
    }

    @Override
    public void showData() {

    }
}
