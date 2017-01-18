package org.firstinspires.ftc.teamcode.org.suffernrobotics.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutonomousOp extends LinearOpMode implements RobotConstants {
    //Creates an instance of a robot, which holds all of the hardware for the robot
    private Robot robot;
    //Method Decleration
    /**
     *
     * @return the team that is autonomous is running on(is overwritten in classes that register this autonomous)
     */
    public abstract ColorType team();

    /**
     *
     * @param power = power of all the motors
     * @param dir = what direction the robot should move in
     * @return a RobotAction that can be run
     */
    public RobotAction SimpleRobotAction(double power,MechanumDirection dir){
        return SimpleRobotAction(power,dir,0);
    }

    /**
     *
     * @param power = power of all the motors
     * @param dir = what direction the robot should move in
     * @param time = amount of time the action should run for
     * @return a RobotAction that can be run
     */
    public RobotAction SimpleRobotAction(double power,MechanumDirection dir,double time){
        return new RobotAction(power*dir.fl, power*dir.fr, power*dir.bl, power*dir.br,time,robot, this);
    }
    public RobotAction SimpleRobotActionWithParam(double power,MechanumDirection dir,double time,ParamType paramType){
        return SimpleRobotAction(power,dir,time).setParam(paramType);
    }
    public RobotAction SimpleRobotActionWithParam(double power,MechanumDirection dir,double time,ParamType paramType,boolean flag){
        return SimpleRobotAction(power,dir,time).setParam(paramType).setFlag(flag);
    }
    public RobotAction SimpleRobotActionWithParam(double power,MechanumDirection dir,double time,ParamType paramType,int data){
        return SimpleRobotAction(power,dir,time).setParam(paramType).setEncoderThreshold(data);
    }

    /**
     *
     * @param position angle for robot to rotate to using the gyro
     */
    public void rotateTo(int position){
        int dir = robot.findBestPath(robot.getGyro().getHeading(),position).getDirection();
        SimpleRobotAction(.2*dir,MechanumDirection.ROTATE).runActionWithoutTime();
        while(Math.abs(robot.getGyro().getHeading()-position)>3&&opModeIsActive()){
            telemetry.addData("Phase","Ajusting");
            telemetry.update();
        }
        SimpleRobotAction(0,MechanumDirection.FORWARD).runActionWithoutTime();
    }

    //AutonomousOp Constructor (Necessary for class to be extended)
    public AutonomousOp(){}

    //this is what actually runs when the robot is initilized
    @Override
    public void runOpMode() {//When the robot has to move side to side, with the mechanum wheels,
        //the motor speed is multiplied by team.getDirection(),because this code is run for both sides
        robot = new Robot(hardwareMap,team(),telemetry);
        while(robot.getGyro().isCalibrating()&&opModeIsActive()){}
        waitForStart(); //Wait for opmode to begin
        SimpleRobotActionWithParam(.5,MechanumDirection.FORWARD,1.8,ParamType.ENCODER,2850).runAction();
        robot.getMotors().get(FRONT_LEFT_MOTOR).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getMotors().get(FRONT_LEFT_MOTOR).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getMotors().get(FRONT_RIGHT_MOTOR).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getMotors().get(BACK_LEFT_MOTOR).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getMotors().get(BACK_RIGHT_MOTOR).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SimpleRobotActionWithParam(.6,MechanumDirection.RIGHT,3,ParamType.ENCODER,6100).setTeamDirection().runAction();
        robot.getMotors().get(FRONT_LEFT_MOTOR).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getMotors().get(FRONT_RIGHT_MOTOR).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getMotors().get(BACK_LEFT_MOTOR).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getMotors().get(BACK_RIGHT_MOTOR).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotateTo(0);
        SimpleRobotAction(0,MechanumDirection.ROTATE,0).runActionWithoutTime();
        while(opModeIsActive()){}
        SimpleRobotAction(.4,MechanumDirection.RIGHT,.2).setTeamDirection().runAction();
        SimpleRobotAction(.2,MechanumDirection.BACKWARD,0.4).runAction();
        //Captures both beacons
        int i=0;
        while(opModeIsActive()&&i<2) {//This code is run twice, once for each beacon
            SimpleRobotActionWithParam(.2,MechanumDirection.FORWARD, 5,ParamType.LINE).runAction();
            SimpleRobotAction(0,MechanumDirection.FORWARD,.5).runAction();
            //Moves backwards before "scanning" for the color of our team on the beacon
            SimpleRobotAction(0.2, MechanumDirection.BACKWARD, 0.2).runAction();
            ElapsedTime time = new ElapsedTime();
            time.reset();
            SimpleRobotActionWithParam(0.1,MechanumDirection.FORWARD,3, ParamType.COLOR,false).runAction();
            double seconds = time.seconds();
            //Goes just past our color, which positions our button pusher to hit the beacon
            SimpleRobotActionWithParam(0.1,MechanumDirection.FORWARD, 3,ParamType.COLOR).runAction();
            robot.getServo().setPosition(-1+team().getDirection());//Goes to position 1 if blue and position 0 if red
            SimpleRobotAction(0,MechanumDirection.FORWARD, 1).runAction();
            //hits the button by moving such that the button pusher drags across the beacon face
            if(seconds>0.3)
                SimpleRobotAction(.1,MechanumDirection.BACKWARD,0.3);
            SimpleRobotAction(0.2,MechanumDirection.RIGHT, 0.5).setTeamDirection().runAction();
            SimpleRobotAction(0.1, MechanumDirection.BACKWARD,3).runAction();
            robot.getServo().setPosition(-1+team().getDirection());//goes to position 0 if blue and 1 if red
            SimpleRobotAction(0,MechanumDirection.FORWARD, 0.5).runAction();
            //Distance robot slightly from field wall
            SimpleRobotAction(0.2,MechanumDirection.LEFT, 0.7).setTeamDirection().runAction();
            rotateTo(0);
            robot.getServo().setPosition((1+team().getDirection())/2);
            //Runs only the first time, in order to ensure our robot passes the first line and sees only the second one
            if (i == 0)
                SimpleRobotAction(0.2, MechanumDirection.FORWARD, 1).runAction();
            i+=1;
        }
    }
}

