package org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class PlayerClass2 extends PlayerClass {
   public PlayerClass2(JoystickActivity joystickActivity, RobotClass.Collector teleskope, OpMode op){
       super(joystickActivity, op.telemetry);


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
