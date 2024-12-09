package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Odometry;
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
        this.teleSkope = robot.teleSkope;
        this.mecanumDrivetrain = robot.drivetrain;
        this.odometry = robot.odometry;
        this.telemetry = robot.metry.getTelemetry();

        if (telemetry != null) {
            telemetry.addLine("Telemetry is Ready!");
        }
    }

    public void showValue(DataTarget target, DataObject object, DataFilter filter) {
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
                                addData(object.name(), odometry.ticksToCm(odometry.getEncL().getCurrentPosition()));
                                break;
                            case ENCR:
                                addData(object.name(), odometry.ticksToCm(odometry.getEncR().getCurrentPosition()));
                                break;
                            case ENCM:
                                addData(object.name(), odometry.ticksToCm(odometry.getEncM().getCurrentPosition()));
                                break;
                            case UPSTANDINGLEFT:
                                addData(object.name() + filter.name(), teleSkope.ticksToCM(teleSkope.getUpStandingLeft().getCurrentPosition()));
                                break;
                            case UPSTANDINGRIGHT:
                                addData(object.name() + filter.name(), teleSkope.ticksToCM(teleSkope.getUpStandingRight().getCurrentPosition()));
                            default:
                                addData(object.name(), "Not exist");
                                break;
                        }
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
                            default:
                                addData(object.name(), "Not exist");
                                break;
                        }
                }
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
                                addData(object.name(), "Not exist");
                                break;
                        }
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
                                addData(object.name(), "Not exist");
                                break;
                        }
                }
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
                                addData(object.name(), "Not exist");
                                break;
                        }
                }
            case displayOtherPosition:
                switch (filter) {
                    case POSITION:
                        switch (object) {
                            case HORIZONTAL:
                                addData(object.name(), teleSkope.getHorizontal().getPosition());
                                break;
                            default:
                                addData(object.name(), "Not exist");
                                break;
                        }
                }
            default:
                addData(object.name(), "No such target data exist");
                break;

        }
    }

    public void showGroupData(DataGroup group, DataTarget target, DataFilter filter){
        switch(group){
            case DRIVETRAIN:
                switch (target){
                    case displayCurPower:
                        showValue(target, DataObject.LEFTB,  filter);
                        showValue(target, DataObject.LEFTF,  filter);
                        showValue(target, DataObject.RIGHTB, filter);
                        showValue(target, DataObject.RIGHTF, filter);
                    default:
                        addData(target.name(), "No such target exist");
                        break;
                }

            case ODOMETRY:
                switch (target){
                    case displayCurPosition:
                        showValue(target, DataObject.ENCL, filter);
                        showValue(target, DataObject.ENCR, filter);
                        showValue(target, DataObject.ENCM, filter);
                    case displayCurVelocity:
                        showValue(target, DataObject.ENCL, filter);
                        showValue(target, DataObject.ENCR, filter);
                        showValue(target, DataObject.ENCM, filter);
                    default:
                        addData(target.name(), "No such target exist");
                        break;
                }

            case TELESKOPE:
                switch (target){
                    case displayCurHeightTeleskope:
                        switch (filter) {
                            case CM:
                                addLine("Height Teleskope (cm)");
                                addData("Height in CM", teleSkope.getHeight());
                                addLine();
                            case TICKS:
                                addLine("Height Teleskope (ticks)");
                                addData("Height in Ticks", teleSkope.cmToTicks(teleSkope.getHeight()));
                                addLine();
                            default:
                                addData(filter.toString(), "No such filter exist");
                                break;
                        }
                    default:
                        addData(target.name(), "No such target exist");
                        break;
                }

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
                            case TICKS:
                                addLine("Robot Global Position (ticks)");
                                addData("GX:", odometry.cmToTicks(odometry.getGlobalPosition().getX()));
                                addData("GY:", odometry.cmToTicks(odometry.getGlobalPosition().getY()));
                                addData("GHeading:", odometry.getGlobalPosition().getHeading());
                                addLine();
                            default:
                                addData(filter.toString(), "No such filter exist");
                                break;
                        }

                    case displayCurVelocity:
                        switch (filter) {
                            case TICKS:
                                addLine("Robot Velocity (ticks)");
                                addData("Velocity:", odometry.cmToTicks(odometry.getSpeed()));
                                addData("VelY:", odometry.cmToTicks(odometry.getVelocity().y));
                                addData("VelX:", odometry.cmToTicks(odometry.getVelocity().x));
                                addLine();
                            case CM:
                                addLine("Robot Velocity (cm)");
                                addData("Velocity:", odometry.getSpeed());
                                addData("VelY:", odometry.getVelocity().y);
                                addData("VelX:", odometry.getVelocity().x);
                                addLine();
                            default:
                                addData(filter.toString(), "No such filter exist");
                                break;
                        }
                    case displayCurAcceleration:
                        switch (filter) {
                            case TICKS:
                                addLine("Robot Acceleration (ticks)");
                                addData("Velocity:", odometry.cmToTicks(odometry.getAcceleration().length()));
                                addData("VelY:", odometry.cmToTicks(odometry.getAcceleration().y));
                                addData("VelX:", odometry.cmToTicks(odometry.getAcceleration().x));
                                addLine();
                            case CM:
                                addLine("Robot Acceleration (cm)");
                                addData("Velocity:", odometry.getAcceleration().length());
                                addData("VelY:", odometry.getAcceleration().y);
                                addData("VelX:", odometry.getAcceleration().x);
                                addLine();
                            default:
                                addData(filter.toString(), "No such filter exist");
                                break;

                        }
                    default:
                        addData(target.name(), "No such target exist");
                        break;
                }
        }
    }

    public void showValueJoystick(DataTarget target, DataObject object, JoystickStatement area) {
        if (target == DataTarget.displayJoystickStateMent) {
            switch (object) {
                case GAMEPAD1:
                    switch (area) {
                        case LEFT_STICK:
                            addData(area.name(), joysticks.getGamepad1().left_stick_x);
                            addData(area.name(), -joysticks.getGamepad1().left_stick_y);
                            addLine();
                        case RIGHT_STICK:
                            addData(area.name(), joysticks.getGamepad1().right_stick_x);
                            addData(area.name(), -joysticks.getGamepad1().right_stick_y);
                            addLine();
                        case A:
                            addData(area.name(), joysticks.getGamepad1().a);
                        case B:
                            addData(area.name(), joysticks.getGamepad1().b);
                        case X:
                            addData(area.name(), joysticks.getGamepad1().x);
                        case Y:
                            addData(area.name(), joysticks.getGamepad1().y);
                        default:
                            addData(area.name(), "Not such area");
                            break;
                    }
                case GAMEPAD2:
                    switch (area) {
                        case LEFT_STICK:
                            addData(area.name() + "X", joysticks.getGamepad2().left_stick_x);
                            addData(area.name() + "Y", joysticks.getGamepad2().left_stick_y);
                            addLine();
                        case RIGHT_STICK:
                            addData(area.name() + "X", joysticks.getGamepad2().right_stick_x);
                            addData(area.name() + "Y", joysticks.getGamepad2().right_stick_y);
                            addLine();
                        case A:
                            addData(area.name(), joysticks.getGamepad2().a);
                        case B:
                            addData(area.name(), joysticks.getGamepad2().b);
                        case X:
                            addData(area.name(), joysticks.getGamepad2().x);
                        case Y:
                            addData(area.name(), joysticks.getGamepad2().y);
                        default:
                            addData(area.name(), "Not such area");
                            break;
                    }
                default:
                    addData(object.name(), "Not such object");
                    break;
            }
        }

    }

    public void addLine(String s) {
        telemetry.addLine(s);
    }

    public void addLine() {
        telemetry.addLine();
    }

    public void addData(String nameofValue, double number) {
        telemetry.addData(nameofValue, number);
    }

    public void addData(String nameofValue, boolean bool) {
        telemetry.addData(nameofValue, bool);
    }

    public void addData(String nameofValue, String s) {
        telemetry.addData(nameofValue, s);
    }

    public void dataForAuto() {
        showGroupData(DataGroup.ROBOT, DataTarget.displayCurPosOnField, DataFilter.CM);
        showGroupData(DataGroup.TELESKOPE ,DataTarget.displayCurHeightTeleskope, DataFilter.CM);
        showValue(DataTarget.displayCurPosition, DataObject.HORIZONTAL, DataFilter.POSITION);
    }

    public void dataForTeleOp() {
        if(isRobotDrive){
            showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD1, JoystickStatement.LEFT_STICK);
            showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD1, JoystickStatement.RIGHT_STICK);
            showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD2, JoystickStatement.LEFT_STICK);
            showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD2, JoystickStatement.RIGHT_STICK);
        }

        showGroupData(DataGroup.ROBOT, DataTarget.displayCurPosOnField, DataFilter.CM);
        showGroupData(DataGroup.TELESKOPE ,DataTarget.displayCurHeightTeleskope, DataFilter.CM);
        showValue(DataTarget.displayCurPosition, DataObject.HORIZONTAL, DataFilter.POSITION);

        showGroupData(DataGroup.DRIVETRAIN, DataTarget.displayCurPower, DataFilter.POWER);
        showGroupData(DataGroup.ROBOT, DataTarget.displayCurVelocity, DataFilter.CM);

        updateRobotMovingState();
    }

    private void updateRobotMovingState(){
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

    public void update(){
        telemetry.update();
    }

    public void killTelemetry(){
        isTelemetryKilled = true;
    }
}
