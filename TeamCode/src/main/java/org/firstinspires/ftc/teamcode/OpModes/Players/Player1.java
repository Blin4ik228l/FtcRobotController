package org.firstinspires.ftc.teamcode.OpModes.Players;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.Odometry;

public class Player1 extends Thread implements Module {
    public Gamepad g1;

    public final Odometry odometry; // Система вычислений одометрии
    public final MecanumDrivetrain drivetrain; // Телега робота
    public final Joysticks joysticks;

    public Player1(OpMode op){
        joysticks = new Joysticks(op, 1);
        drivetrain = new MecanumDrivetrain(op);
        odometry = new Odometry(op);
    }

    @Override
    public void init() {
        joysticks.init();
        drivetrain.init();
        odometry.init();
    }

    public void startTeleop(){
        this.setDaemon(true);
        this.start();
    }

    @Override
    public void run() {
        while (!this.isInterrupted()){
            teleopPl1();
        }
    }

    // Gamepad 1
    public synchronized void teleopPl1() {
        g1 = joysticks.getGamepad1();

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
}
