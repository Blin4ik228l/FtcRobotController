package org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class PlayerClass2 extends PlayerClass {
   public PlayerClass2(JoystickActivityClass joystickActivityClass, RobotClass.Collector teleskope, OpMode op){
       super(joystickActivityClass, op.telemetry);


    }
    public RobotClass.Collector collector;
    public JoystickActivityClass joystickActivityClass;
    public AutoPlayerClass autoPlayerClass;

    @Override
    public void execute() {
    }

    @Override
    public void showData() {

    }
}
