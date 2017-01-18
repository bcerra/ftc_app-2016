package org.firstinspires.ftc.teamcode.org.suffernrobotics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ServoDebug", group = "OpModes")
public class DebugServoProgram extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo servo1= hardwareMap.servo.get("servo_1");
        Servo servo2= hardwareMap.servo.get("servo_2");
        waitForStart();
        servo1.setPosition(1);
        servo2.setPosition(.923);
        ElapsedTime runtime=new ElapsedTime();
        while(opModeIsActive()&&runtime.seconds()<10){
        }
        servo1.setPosition(.923);//.77 is one    roation
        servo2.setPosition(1);//.77 is one roation
        runtime.reset();
        while(opModeIsActive()&&runtime.seconds()<10){
        }
    }
}

