package org.firstinspires.ftc.teamcode.org.suffernrobotics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "TeleOpOldRobot", group = "OpModes")
@SuppressWarnings({"unused", "SuspiciousNameCombination"})
public class ActualRobotStuff extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    //Variable creation
    Servo servo, servo2;
    DcMotor motorFL, motorFR, motorBL, motorBR;
    double servopos = 0;

    @Override
    public void init() {
        //Initialize servos
        servo = hardwareMap.servo.get("servo_1");
        servo2 = hardwareMap.servo.get("servo_2");
        //Initialize motors
        motorFL = hardwareMap.dcMotor.get("motor_1");
        motorFR = hardwareMap.dcMotor.get("motor_2");
        motorBL = hardwareMap.dcMotor.get("motor_3");
        motorBR = hardwareMap.dcMotor.get("motor_4");
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up)
            servopos+=0.05;
        if(gamepad1.dpad_down)
            servopos-=0.05;
        servopos = Range.clip(servopos, 0, 1);
        //Set the position of the servos
        servo.setPosition(1 - servopos);
        servo2.setPosition((servopos));
        //SERVO CODE END
        //Drive Begin
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rotatedX = (Math.sqrt(2) / 2) * (x + y);
        double rotatedY = (Math.sqrt(2) / 2) * (-x + y);
        double x2 = Math.pow(rotatedX, 2);
        double y2 = Math.pow(rotatedY, 2);
        double cts = Math.sqrt((x2 + y2) / Math.max(x2, y2));
        double xSquared = rotatedX * cts;
        double ySquared = rotatedY * cts;
        if (!Double.isNaN(cts)) {
            motorFL.setPower(-xSquared);
            motorFR.setPower(ySquared);
            motorBL.setPower(-ySquared);
            motorBR.setPower(xSquared);
        } else if (gamepad1.right_stick_x < - 0.1) {
            motorFL.setPower(-1);
            motorFR.setPower(-1);
            motorBL.setPower(-1);
            motorBR.setPower(-1);
        } else if(gamepad1.right_stick_x > 0.1) {
            motorFL.setPower(1);
            motorFR.setPower(1);
            motorBL.setPower(1);
            motorBR.setPower(1);
        }else{
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}

}
