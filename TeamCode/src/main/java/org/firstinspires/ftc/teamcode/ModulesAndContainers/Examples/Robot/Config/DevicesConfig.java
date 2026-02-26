package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config;

public abstract class DevicesConfig {
    //моторы
    private String motor0 = "";
    private String encoder0 = motor0;
    private String motor1 = "";
    private String encoder1 = motor1;
    private String motor2 = "";
    private String encoder2 = motor2;
    private String motor3 = "";
    private String encoder3 = motor3;
    //серваки
    private String servo0 = "";
    private String servo1 = "";
    private String servo2 = "";
    private String servo3 = "";
    private String servo4 = "";
    private String servo5 = "";
    //вписать вместо deviceName - тип устройства (colorSensor, distance...)
    //I2C
    private String i2cDeviceName0 = "";
    private String i2cDeviceName1 = "";
    private String i2cDeviceName2 = "";
    private String i2cDeviceName3 = "";
    //Digital
    private String digDeviceName01 = "";
    private String digDeviceName23 = "";
    private String digDeviceName45 = "";
    private String digDeviceName67 = "";
    //Analog
    private String analogDeviceName01 = "";
    private String analogDeviceName23 = "";

    public String getMotor(int port){
        switch (port){
            case 0:
                return motor0;
            case 1:
                return motor1;
            case 2:
                return motor2;
            case 3:
                return motor3;
            default:
                return "";
        }
    }
    public String getEncoder(int port){
        switch (port){
            case 0:
                return encoder0;
            case 1:
                return encoder1;
            case 2:
                return encoder2;
            case 3:
                return encoder3;
            default:
                return "";
        }
    }
    public String getServo0(int port){
        switch (port){
            case 0:
                return servo0;
            case 1:
                return servo1;
            case 2:
                return servo2;
            case 3:
                return servo3;
            case 4:
                return servo4;
            case 5:
                return servo5;
            default:
                return "";
        }
    }
    public String getI2C(int port){
        switch (port){
            case 0:
                return i2cDeviceName0;
            case 1:
                return i2cDeviceName1;
            case 2:
                return i2cDeviceName2;
            case 3:
                return i2cDeviceName3;
            default:
                return "";
        }
    }
    public String getDigital(int ch){
        switch (ch){
            case 0:
                return digDeviceName01;
            case 1:
                return digDeviceName01;
            case 2:
                return digDeviceName23;
            case 3:
                return digDeviceName23;
            case 4:
                return digDeviceName45;
            case 5:
                return digDeviceName45;
            case 6:
                return digDeviceName67;
            case 7:
                return digDeviceName67;
            default:
                return "";
        }
    }
    public String getAnalog(int ch){
        switch (ch){
            case 0:
                return analogDeviceName01;
            case 1:
                return analogDeviceName01;
            case 2:
                return analogDeviceName23;
            case 3:
                return analogDeviceName23;
            default:
                return "";
        }
    }

    public class Builder{
        public Builder setMotorNames(String...motorNames){
            int count = 0;
            for (String name:motorNames) {
                switch (count){
                    case 0:
                        motor0 = name;
                        break;
                    case 1:
                        motor1 = name;
                        break;
                    case 2:
                        motor2 = name;
                        break;
                    case 3:
                        motor3 = name;
                        break;
                }
                count++;
            }
            return this;
        }
        public Builder setServoNames(String...servoNames){
            int count = 0;
            for (String name:servoNames) {
                switch (count){
                    case 0:
                        servo0 = name;
                        break;
                    case 1:
                        servo1 = name;
                        break;
                    case 2:
                        servo2 = name;
                        break;
                    case 3:
                        servo3 = name;
                        break;
                    case 4:
                        servo4 = name;
                        break;
                    case 5:
                        servo5 = name;
                        break;
                }
                count++;
            }
            return this;
        }
        public Builder setI2CDeviceNames(String...i2cDeviceNames){
            int count = 0;
            for (String name:i2cDeviceNames) {
                switch (count){
                    case 0:
                        i2cDeviceName0 = name;
                        break;
                    case 1:
                        i2cDeviceName1 = name;
                        break;
                    case 2:
                        i2cDeviceName2 = name;
                        break;
                    case 3:
                        i2cDeviceName3 = name;
                        break;
                }
                count++;
            }
            return this;
        }
        public Builder setDigDeviceNames(String...digDeviceNames){
            int count = 0;
            for (String name:digDeviceNames) {
                switch (count){
                    case 0:
                        digDeviceName01 = name;
                        break;
                    case 1:
                        digDeviceName23 = name;
                        break;
                    case 2:
                        digDeviceName45 = name;
                        break;
                    case 3:
                        digDeviceName67 = name;
                        break;
                }
                count++;
            }
            return this;
        }
        public Builder setAnalogDeviceNames(String...analogDeviceNames){
            int count = 0;
            for (String name:analogDeviceNames) {
                switch (count){
                    case 0:
                        analogDeviceName01 = name;
                        break;
                    case 1:
                        analogDeviceName23 = name;
                        break;
                }
                count++;
            }
            return this;
        }
        public void endWrite(){}
    }
}
