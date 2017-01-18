package org.firstinspires.ftc.teamcode.org.suffernrobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp", group = "OpModes")
public class LinearTeleOp extends LinearOpMode{

    //Variable creation
    Servo leftServo, rightServo;
    DcMotor motorFL, motorFR, motorBL, motorBR,motorSW,motorSH;
    double rPos, lPos = 0;

    //returns zero if there is a math error in the input, otherwise returns the input
    public double makeANumber(double number) {
        if (Double.isNaN(number))
            return 0;
        return number;
    }

    public void maphardware() {
        //Initialize servos
        rightServo = hardwareMap.servo.get("servo_1");
        leftServo = hardwareMap.servo.get("servo_2");
        //Initialize motors
        motorFL = hardwareMap.dcMotor.get("motor_1");
        motorFR = hardwareMap.dcMotor.get("motor_2");
        motorBL = hardwareMap.dcMotor.get("motor_3");
        motorBR = hardwareMap.dcMotor.get("motor_4");
        motorSW = hardwareMap.dcMotor.get("motor_5");
        motorSH = hardwareMap.dcMotor.get("motor_6");
        //Makes sure motors are initilized to move in the correct direction(for some autonomous
        // programs, the direction is changed)
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void runOpMode() {
        maphardware();
        waitForStart();
        while (opModeIsActive()) {
            //Increases or decreases the variable for the locally stored position of the servo based
            //on the gamepad input
            rPos -= gamepad2.right_stick_x/50;
            lPos += gamepad2.left_stick_x/50;
            //Makes sure that both positions are within the 0-1 range of the servo, and if not,
            //fixes it
            rPos = Range.clip(rPos, 0, 1);
            lPos = Range.clip(lPos, 0, 1);
            //Set the position of the servos to the locally stored variable(.923 and .077 is because
            // we are utilizing winch servos)
            rightServo.setPosition(1-(rPos*.077));
            leftServo.setPosition(.923+(lPos*.077));
            //Gets gamepad input and applies necessary math functions in order to make input useable
            //by the mechanum wheels
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rotatedX = (Math.sqrt(2) / 2) * (x + y);
            double rotatedY = (Math.sqrt(2) / 2) * (-x + y);
            //Scales the values such that it is possible for the motor values to go at max speed
            double x2 = Math.pow(rotatedX, 2);
            double y2 = Math.pow(rotatedY, 2);
            double cts = makeANumber(Math.sqrt((x2 + y2) / Math.max(x2, y2)));
            double xScaled = rotatedX * cts;
            double yScaled = rotatedY * cts;
            //Gets joystick 2 input
            double turn = gamepad1.right_stick_x;
            //Sets the power of each motor based on the input which the math has been applied to
            motorFL.setPower(((yScaled)) + (turn));
            motorFR.setPower(((-xScaled)) + (turn));
            motorBL.setPower(((xScaled)) + (turn));
            motorBR.setPower(((-yScaled)) + (turn));
            //Move sweeper based on trigger inputs
            motorSW.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            //move sweeper if the x button is pressed
            if(gamepad1.x) {
                motorSH.setPower(-1);
            }else{
                motorSH.setPower(0);
            }
        }
    }
}

