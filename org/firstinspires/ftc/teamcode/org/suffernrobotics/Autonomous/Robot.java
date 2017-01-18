package org.firstinspires.ftc.teamcode.org.suffernrobotics.Autonomous;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Robot implements RobotConstants {
    //Physical Component Decleration
    private OpticalDistanceSensor ods;
    private ColorSensor colorSensor;
    private Servo servo;
    private GyroSensor gyro;
    //Other Variable Definition
    private double noLine = 0;
    private ColorType team;
    private Telemetry telemetry;
    private HashMap<String,DcMotor> dcMotorHashMap;
    public Robot(HardwareMap hardwareMap,ColorType team,Telemetry telemetry) {
        this.telemetry=telemetry;
        this.team=team;
        //Initilize Sensors
        colorSensor = hardwareMap.colorSensor.get("color_"+team.value);
        colorSensor.enableLed(false);
        ods = hardwareMap.opticalDistanceSensor.get("ods_1");
        gyro = hardwareMap.gyroSensor.get("gyro_1");
        gyro.calibrate();
        //Initialize and store motors
        dcMotorHashMap=new HashMap<>();
        dcMotorHashMap.put(FRONT_LEFT_MOTOR,hardwareMap.dcMotor.get("motor_1"));
        dcMotorHashMap.put(FRONT_RIGHT_MOTOR,hardwareMap.dcMotor.get("motor_2"));
        dcMotorHashMap.put(BACK_LEFT_MOTOR,hardwareMap.dcMotor.get("motor_3"));
        dcMotorHashMap.put(BACK_RIGHT_MOTOR,hardwareMap.dcMotor.get("motor_4"));
        //Initilize Servos
        servo = hardwareMap.servo.get("servo_"+team.value);
        //Get threshold value for the optical distance sensor
        noLine = ods.getLightDetected();
    }
    /**
     * @return greatest color seen by the beacon or undefined if green is seen or color is unclear
     */
    public ColorType getMaxColorFrequency() {
        int minColorThreshold = 2;
        // if color sensor does not give clear reading then return undefined.
        if (colorSensor.red() < minColorThreshold && colorSensor.green() < minColorThreshold && colorSensor.blue() < minColorThreshold)
            return ColorType.UNDEFINED;

        if (colorSensor.red() > colorSensor.blue()) {
            if (colorSensor.green() > colorSensor.red())
                return ColorType.UNDEFINED;
            else
                return ColorType.RED;
        } else if (colorSensor.blue() > colorSensor.red()) {
            if (colorSensor.green() > colorSensor.blue())
                return ColorType.UNDEFINED;
            else
                return ColorType.BLUE;
        } else
            return ColorType.UNDEFINED;
    }
    /**
     * @return whether or not the Optical Distance Sensor can see a line, based off the value
     * recorded at the beginning of the match
     */
    public boolean visibleLine() {
        //NoLine, the threshold value is assigned to the amount of light detected at the beginning
        //of the match in order to ensure that the optical distance sensor works with ambient light
        if (ods.getLightDetected() > 2 * noLine)
            return true;
        return false;
    }

    //Finds which way the robot should rotate to efficiently line it'self up with the end angle
    public ColorType findBestPath(int start,int end){
        //ColorType's can be used to represent Colors AND directions
        int distance = end-start;
        if(distance>180)
            distance-=360;
        if(distance>0)
            return ColorType.BLUE;
        if(distance<0)
            return ColorType.RED;
        return ColorType.UNDEFINED;
    }
    //Getter Methods
    public ColorType getTeam(){
        return this.team;
    }
    public Telemetry getTelemetry(){
        return this.telemetry;
    }
    public GyroSensor getGyro(){
        return this.gyro;
    }
    public Servo getServo(){
        return this.servo;
    }
    public HashMap<String,DcMotor> getMotors(){
        return dcMotorHashMap;
    }

}
