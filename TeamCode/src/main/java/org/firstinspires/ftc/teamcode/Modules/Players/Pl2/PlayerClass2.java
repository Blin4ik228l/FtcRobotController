package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class PlayerClass2 extends PlayerClass {
   public PlayerClass2(Gamepad gamepad, RobotClass.Collector teleskope, OpMode op){
       super(new JoystickActivity(gamepad, op.telemetry), op.telemetry);


    }
    public RobotClass.Collector collector;
    public JoystickActivity joystickActivity;
    public AutomaticClass automaticClass;

    @Override
    public void execute() {
    }

    @Override
    public void showData() {

    }
}
