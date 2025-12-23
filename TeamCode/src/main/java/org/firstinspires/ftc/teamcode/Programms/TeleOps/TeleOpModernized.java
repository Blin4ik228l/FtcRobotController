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

        modulePlayer1 = player1;
        moduleInnerWarden = innerWarden;
        moduleAutomatic = autoPlayerClass;

        positionFireLogic = new PositionFireLogic(robot.driveTrain, this);
    }

    public void updateAll() {
        moduleJoystickActivityPlayer1.update();
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

        executeAll();

        showAll();
        telemetry.addData("t", time.seconds());
        time.reset();
    }


    @Override
    public void stop() {

    }
}

