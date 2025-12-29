package org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.Collector;

public class PlayerClass2 extends PlayerClass {
   public PlayerClass2(JoystickActivityClass joystickActivityClass, Collector teleskope, OpMode op){
       super(joystickActivityClass, op.telemetry);


    }
    public Collector collector;
    public JoystickActivityClass joystickActivityClass;
    public AutoPlayerClass autoPlayerClass;

    @Override
    public void execute() {
    }

    @Override
    public void showData() {

    }
}
