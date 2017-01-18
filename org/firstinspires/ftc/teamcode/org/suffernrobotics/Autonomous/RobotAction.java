package org.firstinspires.ftc.teamcode.org.suffernrobotics.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

//Creates a class which represents a RobotAction
//The class alows for easy and quick manipulation of how the robot acts during autonomous
//in order to allow for quick and efficient testing and optimization of the autonomous
public class RobotAction implements RobotConstants {
    //id is printed in telemetry for debug purposes
    private int sID = 1;
    private int id;
    //The following four variables represent the powers of the four motors for this maneuver
    private double FLpower;
    private double BLpower;
    private double FRpower;
    private double BRpower;
    //the amount of time that this manuver will run for
    private double time;
    private Robot robot;
    //Creates a variable used in order to track the amount of time the manuever runs for
    private ElapsedTime runtime = new ElapsedTime();
    private ParamType paramType=null;
    private boolean flag=true;
    private LinearOpMode linearOpMode;
    private int dir=1;//Represents direction(default 1)
    private int encoderThreshold;

    //Robot Action Constructor
    public RobotAction(double FLpower, double BLpower, double FRpower, double BRpower, double time, Robot robot, AutonomousOp autonomousOp) {
        this(FLpower,BLpower,FRpower,BRpower,time,robot,null,true,autonomousOp);//Calls the below function, with a param that always returns true(negligable)
    }
    public RobotAction(double FLpower, double BLpower, double FRpower, double BRpower, double time,Robot robot,ParamType paramType, boolean flag,AutonomousOp autonomousOp){
        //The following two lines assignes this manuver a unique id, for debugging purposes
        this.id = sID;
        sID += 1;
        //Other Variable assignments
        this.FLpower = FLpower;
        this.FRpower = FRpower;
        this.BLpower = BLpower;
        this.BRpower = BRpower;
        this.robot=robot;
        this.time = time;
        this.paramType=paramType;
        this.flag=flag;
        this.linearOpMode=autonomousOp;
    }
    public RobotAction setEncoderThreshold(int threshold){
        this.encoderThreshold=threshold;
        return this;
    }
    //Used when the robot has to go different directions based on the color of the beacon
    public RobotAction setTeamDirection(){
        this.dir= robot.getTeam().getDirection();
        return this;
    }
    public RobotAction setParam(ParamType paramType){
        this.paramType=paramType;
        return this;
    }
    public RobotAction setFlag(boolean flag){
        this.flag=flag;
        return this;
    }
    public boolean ParamLogic(ParamType ParamType,boolean flag,int data) {
        //The ParamType enum(defined earlier) decides what this paremeter is checking for
        //In order to add a new condition, a new case must be added, as well as the enum name
        if(paramType!=null)
        switch (ParamType) {
            case LINE:
                return flag == !robot.visibleLine();
            case COLOR:
                return flag == robot.getMaxColorFrequency().equals(robot.getTeam());
            case ENCODER:
                return flag == robot.getMotors().get(FRONT_LEFT_MOTOR).getCurrentPosition()<data;

        }
        return true;
    }

    //This method makes the robot execute the manuver that this class represents
    void runAction() {
        runtime.reset();
        HashMap<String,DcMotor> motors = robot.getMotors();
        motors.get(FRONT_LEFT_MOTOR).setPower(FLpower*dir);
        motors.get(FRONT_RIGHT_MOTOR).setPower(FRpower*dir);
        motors.get(BACK_LEFT_MOTOR).setPower(BLpower*dir);
        motors.get(BACK_RIGHT_MOTOR).setPower(BRpower*dir);
        while (runtime.seconds() < time && linearOpMode.opModeIsActive() && ParamLogic(paramType,flag,encoderThreshold)) {
            robot.getTelemetry().addData("Phase", id);
            robot.getTelemetry().update();
        }
    }
    void runActionWithoutTime() {
        runtime.reset();
        HashMap<String,DcMotor> motors = robot.getMotors();
        motors.get(FRONT_LEFT_MOTOR).setPower(FLpower*dir);
        motors.get(FRONT_RIGHT_MOTOR).setPower(FRpower*dir);
        motors.get(BACK_LEFT_MOTOR).setPower(BLpower*dir);
        motors.get(BACK_RIGHT_MOTOR).setPower(BRpower*dir);
    }

}