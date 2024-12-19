package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.ServosService;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.TeleSkope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataFilter;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataGroup;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataObject;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataTarget;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.JoystickStatement;

public class DataDisplayer implements Module {
    private final Robot robot;

    private Odometry odometry;
    private MecanumDrivetrain mecanumDrivetrain;
    private TeleSkope teleSkope;
    private Telemetry telemetry;
    private Joysticks joysticks;
    private ServosService servosService;

    boolean isTelemetryKilled = false;
    boolean isRobotDrive = false;
    boolean isTicksToCm = false;

    boolean pastState, switchable = false;
    int released = 0;

    public DataDisplayer(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        this.joysticks = robot.joysticks;
        this.servosService = robot.servosService;
        this.teleSkope = robot.teleSkope;
        this.mecanumDrivetrain = robot.drivetrain;
        this.odometry = robot.odometry;
        this.telemetry = robot.metry.getTelemetry();

        if (telemetry != null) {
            telemetry.addLine("Telemetry is Ready!");
        }
    }

    public synchronized void showValue(DataTarget target, DataObject object, DataFilter filter) {
        if(target == null){
            addLine("target - null");
            return;
        }
        if(object == null){
            addLine("object - null");
            return;
        }
        if (filter == null){
            addLine("filter - null");
            return;
        }
        switch (target) {
            case displayCurPosition:
                switch (filter) {
                    case CM:
                        switch (object) {
                            case ENCL:
                                addData(DataObject.ENCL.name(), odometry.ticksToCm(odometry.getEncL().getCurrentPosition()));
                                break;
                            case ENCR:
                                addData(DataObject.ENCR.name(), odometry.ticksToCm(odometry.getEncR().getCurrentPosition()));
                                break;
                            case ENCM:
                                addData(object.toString(), odometry.ticksToCm(odometry.getEncM().getCurrentPosition()));
                                break;
                            case UPSTANDINGLEFT:
                                addData(object.toString() + filter.toString(), teleSkope.ticksToCM(teleSkope.getUpStandingLeft().getCurrentPosition()));
                                break;
                            case UPSTANDINGRIGHT:
                                addData(object.toString() + filter.toString(), teleSkope.ticksToCM(teleSkope.getUpStandingRight().getCurrentPosition()));
                                break;
                            default:
                                addData(object.toString(), "Not exist");
                                break;
                        }
                        break;
                    case TICKS:
                        switch (object) {
                            case ENCL:
                                addData(object.name(), odometry.getEncL().getCurrentPosition());
                                break;
                            case ENCR:
                                addData(object.name(), odometry.getEncR().getCurrentPosition());
                                break;
                            case ENCM:
                                addData(object.name(), odometry.getEncM().getCurrentPosition());
                                break;
                            case UPSTANDINGLEFT:
                                addData(object.name() + filter.name(), teleSkope.getUpStandingLeft().getCurrentPosition());
                                break;
                            case UPSTANDINGRIGHT:
                                addData(object.name() + filter.name(), teleSkope.getUpStandingRight().getCurrentPosition());
                                break;
                            default:
                                addData(object.name(), "Not exist");
                                break;
                        }
                        break;
                    default:
                        addData(filter.name(), "No such filter exist");
                        break;
                }
                break;
            case displayCurVelocity:
                switch (filter) {
                    case CM:
                        switch (object) {
                            case ENCL:
                                addData(object.name(), odometry.ticksToCm(odometry.getEncL().getVelocity()));
                                break;
                            case ENCR:
                                addData(object.name(), odometry.ticksToCm(odometry.getEncR().getVelocity()));
                                break;
                            case ENCM:
                                addData(object.name(), odometry.ticksToCm(odometry.getEncM().getVelocity()));
                                break;
                            default:
                                addData(object.name(), "This object not exist");
                                break;
                        }
                        break;
                    case TICKS:
                        switch (object) {
                            case ENCL:
                                addData(object.name(), odometry.getEncL().getVelocity());
                                break;
                            case ENCR:
                                addData(object.name(), odometry.getEncR().getVelocity());
                                break;
                            case ENCM:
                                addData(object.name(), odometry.getEncM().getVelocity());
                                break;
                            default:
                                addData(object.name(), "This object not exist");
                                break;
                        }
                        break;
                    default:
                        addData( filter.name(), "No such filter exist");
                        break;
                }
                break;
            case displayCurPower:
                switch (filter) {
                    case POWER:
                        switch (object) {
                            case LEFTB:
                                addData(object.name(), mecanumDrivetrain.leftB.getPower());
                                break;
                            case RIGHTB:
                                addData(object.name(), mecanumDrivetrain.rightB.getPower());
                                break;
                            case LEFTF:
                                addData(object.name(), mecanumDrivetrain.leftF.getPower());
                                break;
                            case RIGHTF:
                                addData(object.name(), mecanumDrivetrain.rightF.getPower());
                                break;
                            case UPSTANDINGLEFT:
                                addData(object.name(), teleSkope.getUpStandingLeft().getPower());
                                break;
                            case UPSTANDINGRIGHT:
                                addData(object.name(), teleSkope.getUpStandingRight().getPower());
                                break;
                            default:
                                addData(object.name(), "This object not exist");
                                break;
                        }
                        break;
                    default:
                        addData(filter.name(), "Filter not exist");
                }
                break;
            case displayOtherPosition:
                switch (filter) {
                    case POSITION:
                        switch (object) {
                            case HORIZONTAL:
                                addData(object.name(), servosService.getHorizontal().getPosition());
                                break;
                            default:
                                addData(object.name(), "Object not exist");
                                break;
                        }
                        break;
                    default:
                        addData(filter.name(), "Filter not exist");
                        break;
                }
                break;
            default:
                addData(target.name(), "No such target data exist");
                break;
        }
    }

    public synchronized void showGroupData(DataGroup group, DataTarget target, DataFilter filter){
        switch(group){
            case DRIVETRAIN:
                switch (target){
                    case displayCurPower:
                        showValue(target, DataObject.LEFTB,  filter);
                        showValue(target, DataObject.LEFTF,  filter);
                        showValue(target, DataObject.RIGHTB, filter);
                        showValue(target, DataObject.RIGHTF, filter);
                        addLine();
                        break;
                    default:
                        addData(group.name() + target.name(), "No such target exist");
                        break;
                }
                break;
            case ODOMETRY:
                switch (target){
                    case displayCurHeightTeleskope:
                        showValue(target, DataObject.ENCL, filter);
                        showValue(target, DataObject.ENCR, filter);
                        showValue(target, DataObject.ENCM, filter);
                        addLine();
                        break;
                    case displayCurVelocity:
                        showValue(target, DataObject.ENCL, filter);
                        showValue(target, DataObject.ENCR, filter);
                        showValue(target, DataObject.ENCM, filter);
                        addLine();
                        break;
                    default:
                        addData(group.name() + target.name(), "No such target exist");
                        break;
                }
                break;

            case TELESKOPE:
                switch (target){
                    case displayCurHeightTeleskope:
                        switch (filter) {
                            case CM:
                                addLine("Height Teleskope (cm)");
                                addData("Height in CM", teleSkope.getHeight());
                                addLine();
                                break;
                            case TICKS:
                                addLine("Height Teleskope (ticks)");
                                addData("Height in Ticks", teleSkope.cmToTicks(teleSkope.getHeight()));
                                addLine();
                                break;
                            default:
                                addData(group.name() + " " + filter.name(), "No such filter exist");
                                break;
                        }
                        break;
                    default:
                        addData(group.name() + " " + target.name(), "Not such target in this group");
                }
                break;
            case ROBOT:
                switch (target){
                    case displayCurPosOnField:
                        switch (filter) {
                            case CM:
                                addLine("Robot Global Position (cm)");
                                addData("GX:", odometry.getGlobalPosition().getX());
                                addData("GY:", odometry.getGlobalPosition().getY());
                                addData("GHeading:", odometry.getGlobalPosition().getHeading());
                                addLine();
                                break;
                            case TICKS:
                                addLine("Robot Global Position (ticks)");
                                addData("GX:", odometry.cmToTicks(odometry.getGlobalPosition().getX()));
                                addData("GY:", odometry.cmToTicks(odometry.getGlobalPosition().getY()));
                                addData("GHeading:", odometry.getGlobalPosition().getHeading());
                                addLine();
                                break;
                            default:
                                addData(filter.toString(), "No such filter exist");
                                break;
                        }
                        break;
                    case displayCurVelocity:
                        switch (filter) {
                            case TICKS:
                                addLine("Robot Velocity (ticks)");
                                addData("Velocity:", odometry.cmToTicks(odometry.getSpeed()));
                                addData("VelY:", odometry.cmToTicks(odometry.getVelocity().y));
                                addData("VelX:", odometry.cmToTicks(odometry.getVelocity().x));
                                addLine();
                                break;
                            case CM:
                                addLine("Robot Velocity (cm)");
                                addData("Velocity:", odometry.getSpeed());
                                addData("VelY:", odometry.getVelocity().y);
                                addData("VelX:", odometry.getVelocity().x);
                                addLine();
                                break;
                            default:
                                addData(filter.toString(), "No such filter exist");
                                break;
                        }
                        break;
                    case displayCurAcceleration:
                        switch (filter) {
                            case TICKS:
                                addLine("Robot Acceleration (ticks)");
                                addData("Acceleration:", odometry.cmToTicks(odometry.getAcceleration().length()));
                                addData("AccelerationY:", odometry.cmToTicks(odometry.getAcceleration().y));
                                addData("AccelerationY:", odometry.cmToTicks(odometry.getAcceleration().x));
                                addLine();
                                break;
                            case CM:
                                addLine("Robot Acceleration (cm)");
                                addData("Acceleration:", odometry.getAcceleration().length());
                                addData("AccelerationY:", odometry.getAcceleration().y);
                                addData("AccelerationX:", odometry.getAcceleration().x);
                                addLine();
                                break;
                            default:
                                addData(filter.toString(), "No such filter exist");
                                break;
                        }
                        break;
                }
                break;
            default:
                addData(group.name(), "No such group exist");
                break;
        }
    }

    public synchronized void showValueJoystick(DataTarget target, DataObject object, JoystickStatement area) {
        if (target == DataTarget.displayJoystickStateMent) {
            switch (object) {
                case GAMEPAD1:
                    switch (area) {
                        case LEFT_STICK:
                            addData(area.name(), joysticks.getGamepad1().left_stick_x);
                            addData(area.name(), -joysticks.getGamepad1().left_stick_y);
                            addLine();
                            break;
                        case RIGHT_STICK:
                            addData(area.name(), joysticks.getGamepad1().right_stick_x);
                            addData(area.name(), -joysticks.getGamepad1().right_stick_y);
                            addLine();
                            break;
                        case A:
                            addData(area.name(), joysticks.getGamepad1().a);
                            break;
                        case B:
                            addData(area.name(), joysticks.getGamepad1().b);
                            break;
                        case X:
                            addData(area.name(), joysticks.getGamepad1().x);
                            break;
                        case Y:
                            addData(area.name(), joysticks.getGamepad1().y);
                            break;
                        default:
                            addData(area.name(), "Not such area");
                            break;
                    }
                    break;
                case GAMEPAD2:
                    switch (area) {
                        case LEFT_STICK:
                            addData(area.name() + "X", joysticks.getGamepad2().left_stick_x);
                            addData(area.name() + "Y", joysticks.getGamepad2().left_stick_y);
                            addLine();
                            break;
                        case RIGHT_STICK:
                            addData(area.name() + "X", joysticks.getGamepad2().right_stick_x);
                            addData(area.name() + "Y", joysticks.getGamepad2().right_stick_y);
                            addLine();
                            break;
                        case A:
                            addData(area.name(), joysticks.getGamepad2().a);
                            break;
                        case B:
                            addData(area.name(), joysticks.getGamepad2().b);
                            break;
                        case X:
                            addData(area.name(), joysticks.getGamepad2().x);
                            break;
                        case Y:
                            addData(area.name(), joysticks.getGamepad2().y);
                            break;
                        default:
                            addData(area.name(), "Not such area");
                            break;
                    }
                    break;
                default:
                    addData(object.name(), "Not such object");
                    break;
            }
        }

    }

    public synchronized void addLine(String s) {
        telemetry.addLine(s);
    }

    public synchronized void addLine() {
        telemetry.addLine();
    }

    public synchronized void addData(String nameofValue, double number) {
        telemetry.addData(nameofValue, number);
    }

    public synchronized void addData(String nameofValue, boolean bool) {
        telemetry.addData(nameofValue, bool);
    }

    public synchronized void addData(String nameofValue, String s) {
        telemetry.addData(nameofValue, s);
    }

    public synchronized void dataForAuto() {
        showGroupData(DataGroup.ROBOT, DataTarget.displayCurPosOnField, DataFilter.CM);
        showGroupData(DataGroup.TELESKOPE ,DataTarget.displayCurHeightTeleskope, DataFilter.CM);
        showValue(DataTarget.displayOtherPosition, DataObject.HORIZONTAL, DataFilter.POSITION);
    }

    public synchronized void dataForTeleOp() {
        if(isRobotDrive){
            showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD1, JoystickStatement.LEFT_STICK);
            showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD1, JoystickStatement.RIGHT_STICK);
            showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD2, JoystickStatement.LEFT_STICK);
            showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD2, JoystickStatement.RIGHT_STICK);
        }

        showGroupData(DataGroup.ROBOT, DataTarget.displayCurPosOnField, DataFilter.CM);
        showGroupData(DataGroup.TELESKOPE ,DataTarget.displayCurHeightTeleskope, DataFilter.CM);
        showValue(DataTarget.displayOtherPosition, DataObject.HORIZONTAL, DataFilter.POSITION);

        showGroupData(DataGroup.DRIVETRAIN, DataTarget.displayCurPower, DataFilter.POWER);
        showGroupData(DataGroup.ROBOT, DataTarget.displayCurVelocity, DataFilter.CM);

        updateRobotMovingState();
    }

    private synchronized void updateRobotMovingState(){
        isRobotDrive = joysticks.getGamepad1().left_stick_y != 0 || joysticks.getGamepad1().right_stick_y != 0 || joysticks.getGamepad1().right_stick_x != 0;
    }

    private void switchTicksToCmState(){

        if(joysticks.getGamepad1().a && joysticks.getGamepad1().y && released == 0) {
            switchable = !switchable;
            released = 1;}

        if(!joysticks.getGamepad1().a && !joysticks.getGamepad1().y && released != 0){
            released = 0;
        }

        isTicksToCm = switchable;
    }

    public synchronized void update(){
        telemetry.update();
    }

    public void killTelemetry(){
        isTelemetryKilled = true;
    }
}
