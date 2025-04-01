package org.firstinspires.ftc.teamcode.Game.Players.MultiPlayer.Driver1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Game.Players.Player;
import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Usable.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Usable.Odometry;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.Utils.Position;

public class Driver1 implements Player {
    /*
    Первый игрок, то же самое, что и первый оператор.
    Отвечает за тележку робота.
    */

    public Driver1(OpMode op){
        this.op = op;

        drivetrain = new MecanumDrivetrain(op);
        odometry = new Odometry(op, new Position(0,0,0));

        joystick1 = new Joystick1(op.gamepad1);
        g1 = joystick1.gamepad;
    }
    public OpMode op;
    public Joystick1 joystick1;
    public Gamepad g1;

    public MecanumDrivetrain drivetrain;
    public Odometry odometry;

    @Override
    public void init() {
        drivetrain.init();
        odometry.init();
    }

    @Override
    public void functional(){
        double max_speed = 0.8;
        double accelLinear = 1.3, accelAngle = 1.3;

        if(g1.left_trigger > 0.05 && g1.right_trigger < 0.05){//Ускорение робота
            accelLinear = 1.8;
            accelAngle = 1.8;
        }

        if(g1.right_trigger > 0.05 && g1.left_trigger < 0.05){//Замедление робота
            accelLinear = 0.25;
            accelAngle = 0.25;
        }

        double forward = -1*g1.left_stick_y;
        double side = g1.left_stick_x;
        double turn = g1.right_stick_x;

        if (Math.abs(forward) < 0.1 && forward != 0){
            forward += 0.1 * Math.signum(forward);
        }
        if (Math.abs(side) < 0.1 && side != 0){
            side += 0.15 * Math.signum(side);
        }

        double forwardVoltage = Range.clip(forward * accelLinear , -max_speed, max_speed);
        double sideVoltage    = Range.clip(side * accelLinear ,  -max_speed, max_speed);
        double angleVoltage   = Range.clip(turn * accelAngle, -max_speed, max_speed);

        drivetrain.setPowerTeleOp(forwardVoltage, sideVoltage, angleVoltage);
    }

    @Override
    public void interruptJoystick() {
        joystick1.interrupt();
    }
}
