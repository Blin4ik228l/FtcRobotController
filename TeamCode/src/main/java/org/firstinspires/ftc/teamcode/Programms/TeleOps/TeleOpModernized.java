package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts.PositionFireLogic;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl0.InnerWardenClass;
import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public abstract class TeleOpModernized extends OpMode {
    public RobotClass robot;
    private PlayerClass1 player1;
    private AutoPlayerClass autoPlayerClass;
    private InnerWardenClass innerWarden;
    private JoystickActivityClass joystickActivityClass;

    private PositionFireLogic positionFireLogic;
    public UpdatableModule moduleJoystickActivityPlayer1, moduleRobot, moduleInnerWarden;
    public ExecutableModule modulePlayer1, moduleAutomatic;

    public void initAfterRobot(){
        joystickActivityClass = new JoystickActivityClass(gamepad1, this);

        player1 = new PlayerClass1(joystickActivityClass, robot.driveTrain, robot.innerTime,this);
        autoPlayerClass = new AutoPlayerClass(joystickActivityClass, robot.collector, robot.innerTime,this);

        innerWarden = new InnerWardenClass(robot, player1, autoPlayerClass, this);

        moduleRobot = robot;

        moduleJoystickActivityPlayer1 = player1.joystickActivityClass;
        joystickActivityClass2 = new JoystickActivityClass(gamepad2, this);

        modulePlayer1 = player1;
        moduleInnerWarden = innerWarden;
        moduleAutomatic = autoPlayerClass;

        positionFireLogic = new PositionFireLogic(robot.driveTrain, this);
    }

    public void updateAll() {
        moduleJoystickActivityPlayer1.update();
        joystickActivityClass2.update();

        moduleRobot.update();
        moduleInnerWarden.update();
    }
    public void executeAll(){
        modulePlayer1.execute();
        moduleAutomatic.execute();
    }
    public void showAll(){
        moduleJoystickActivityPlayer1.showData();

        moduleRobot.showData();

        moduleAutomatic.showData();
    }
    JoystickActivityClass joystickActivityClass2;
//    private double  P = 20.5, I, D = 1.5, F = 0.23;
    private double  P = 19, I = 0.11, D = 3.0, F = 0.41;
    private double[] stepSize = {1, 0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex;
    private int index;
public ElapsedTime time = new ElapsedTime();
    @Override
    public void init_loop() {
        updateAll();
        showAll();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        updateAll();
        if(joystickActivityClass2.bumperLeft){
            stepIndex = (stepIndex + 1) % stepSize.length;
            joystickActivityClass2.bumperLeft = false;
        }

        if(joystickActivityClass2.bumperRight){
            stepIndex = Math.max(stepIndex - 1, 0);
            joystickActivityClass2.bumperRight = false;
        }

        if(joystickActivityClass2.triggerLeft){
            index = (index + 1) % 4;
            joystickActivityClass2.triggerLeft = false;
        }

        if(joystickActivityClass2.triggerRight){
            index = Math.max(index - 1, 0);
            joystickActivityClass2.triggerRight = false;
        }

        if(joystickActivityClass2.dpad_Up){
            switch (index){
                case 0:
                    P += stepSize[stepIndex];
                    break;
                case 1:
                    I += stepSize[stepIndex];
                    break;
                case 2:
                    D += stepSize[stepIndex];
                    break;
                case 3:
                    F += stepSize[stepIndex];
                    break;
            }
            joystickActivityClass2.dpad_Up = false;
        }

        if(joystickActivityClass2.dpad_Down){
            switch (index){
                case 0:
                    P = Math.max(P - stepSize[stepIndex], 0);
                    break;
                case 1:
                    I = Math.max(I - stepSize[stepIndex], 0);
                    break;
                case 2:
                   D = Math.max(D - stepSize[stepIndex], 0);
                    break;
                case 3:
                    F = Math.max(F - stepSize[stepIndex], 0);
                    break;
            }
            joystickActivityClass2.dpad_Down = false;
        }

//        robot.collector.motors.setPIDF(P, I, D, F);
        executeAll();

        showAll();
        telemetry.addData("P: ", P);
        telemetry.addData("I: ", I);
        telemetry.addData("D: ", D);
        telemetry.addData("F: ", F);
        telemetry.addData("Step index and size", "in: %s sz: %s", stepIndex, stepSize[stepIndex]);
        telemetry.addData("Index", index);
        telemetry.addData("t", time.seconds());
        time.reset();
    }


    @Override
    public void stop() {

    }
}

