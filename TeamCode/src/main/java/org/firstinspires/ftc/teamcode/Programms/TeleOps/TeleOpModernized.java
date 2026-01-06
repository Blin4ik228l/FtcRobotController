package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts.PositionFireLogic;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl0.InnerWardenClass;
import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.TelemetrySettings;

public abstract class TeleOpModernized extends OpMode {
    public RobotClass robot;
    public PlayerClass1 player1;
    public AutoPlayerClass autoPlayerClass;
    public InnerWardenClass innerWarden;
    public JoystickActivityClass joystickActivityClass;
    public JoystickActivityClass joystickActivityClass2;
    public PositionFireLogic positionFireLogic;
    public TelemetrySettings telemetrySettings;
    public ElapsedTime updateTime;
    public int iterationCount = 1;

    public void initAfterRobot(){
        joystickActivityClass = new JoystickActivityClass(gamepad1, this);
        joystickActivityClass2 = new JoystickActivityClass(gamepad2, this);

        player1 = new PlayerClass1(joystickActivityClass, robot.driveTrain, robot.innerRunTime,this);
        autoPlayerClass = new AutoPlayerClass(joystickActivityClass, robot.collector, robot.innerRunTime,this);

        innerWarden = new InnerWardenClass(robot, player1, autoPlayerClass, this);

        positionFireLogic = new PositionFireLogic(robot.driveTrain, this);

        telemetrySettings = new TelemetrySettings(this, null, telemetry);

        robot.collector.motors.setPreferences(CollectorMotors.ControlMode.By_speed, CollectorMotors.Units.Rad_in_sec);

        updateTime = new ElapsedTime();
    }
    public void extUpdate(){

    }
    public void extExecute(){

    }
    public void extShow(){

    }
    private void setAll(){
        joystickActivityClass.setIterationCount(iterationCount);
        joystickActivityClass2.setIterationCount(iterationCount);
        robot.setIterationCount(iterationCount);
    }
    private void updateAll() {
        joystickActivityClass.update();
        joystickActivityClass2.update();

        telemetrySettings.update();

        robot.update();
        innerWarden.update();

        extUpdate();
    }
    private void executeAll(){
        player1.execute();
        autoPlayerClass.execute();

        extExecute();
    }
    private void showAll(){
        telemetrySettings.showData();
        telemetry.addData("Update time / Frequency", "%.2f sec %.2f Hz", updateTime.seconds(), 1 / updateTime.seconds());
    }
    @Override
    public void init_loop() {
        setAll();
        updateAll();
        showAll();
    }

    @Override
    public void start() {
        robot.startTimer();
    }

    @Override
    public void loop() {
        setAll();
        updateAll();
        executeAll();
        showAll();

        iterationCount++;
        updateTime.reset();
    }

    @Override
    public void stop() {

    }
}

