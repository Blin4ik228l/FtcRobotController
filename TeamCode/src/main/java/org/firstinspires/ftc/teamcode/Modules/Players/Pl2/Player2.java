package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Players.Player;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class Player2 extends Player implements Runnable {
   public Player2(Gamepad gamepad, RobotClass.Collector teleSkope, OpMode op){
       super(op.telemetry);

        playersGamepad = gamepad;
        this.teleSkope = teleSkope;
        joystickActivity = new JoystickActivity();
    }
    public RobotClass.Collector teleSkope;
    public JoystickActivity joystickActivity;

    @Override
    public void run() {
        play();
    }

    @Override
    public void play() {
        joystickActivity.checkActivity();

        double leftStickY = -playersGamepad.left_stick_y;

        double upStandingVel = -playersGamepad.right_stick_y;

        double ulitkaPos = 0;
        double rampPos = 0;
        double barabanPos = 0;


        if(joystickActivity.buttonA){
            barabanPos = 0.5;
        }
        if(joystickActivity.buttonB){
            rampPos = 0.43;
        }
        if(joystickActivity.buttonY){
            ulitkaPos = 0.65;
        }
        teleSkope.setTelescope(upStandingVel, leftStickY, ulitkaPos, rampPos, barabanPos);

//        showData();
    }

    @Override
    public void showData() {
        teleSkope.motors.selfData.showHeight();
        teleSkope.servos.showServosPos();
    }
}
