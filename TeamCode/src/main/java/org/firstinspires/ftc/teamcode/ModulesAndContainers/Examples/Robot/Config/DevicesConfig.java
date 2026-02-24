package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config;

public abstract class DevicesConfig {
    //моторы
    public String motor0 = "";
    public String encoder0 = motor0;
    public String motor1 = "";
    public String encoder1 = motor1;
    public String motor2 = "";
    public String encoder2 = motor2;
    public String motor3 = "";
    public String encoder3 = motor3;
    //серваки
    public String servo0 = "";
    public String servo1 = "";
    public String servo2 = "";
    public String servo3 = "";
    public String servo4 = "";
    public String servo5 = "";
    //вписать вместо deviceName - тип устройства (colorSensor, distance...)
    //I2C
    public String i2cDeviceName0 = "";
    public String i2cDeviceName1 = "";
    public String i2cDeviceName2 = "";
    public String i2cDeviceName3 = "";
    //Digital
    public String digDeviceName01 = "";
    public String digDeviceName23 = "";
    public String digDeviceName45 = "";
    public String digDeviceName67 = "";
    //Analog
    public String analogDeviceName01 = "";
    public String analogDeviceName23 = "";

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
