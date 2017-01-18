package org.firstinspires.ftc.teamcode.org.suffernrobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by robotics1 on 1/13/17.
 */

@TeleOp(name = "TeleOpe", group = "OpModes")
public class TeleOpe extends LinearOpMode {

    DcMotor a, b, c, d, sweep, flick, lift;
    Servo left, right;

    public void initHardware() {
        //Initialize servos
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");

        //Initialize motors
        a = hardwareMap.dcMotor.get("a");
        b = hardwareMap.dcMotor.get("b");
        c = hardwareMap.dcMotor.get("c");
        d = hardwareMap.dcMotor.get("d");
        sweep = hardwareMap.dcMotor.get("s");
        flick = hardwareMap.dcMotor.get("f");
        lift = hardwareMap.dcMotor.get("l");

    }

    public double makeANumber(double number) {
        if (Double.isNaN(number))
            return 0;
        return number;
    }

    @Override
    public void runOpMode(){
        initHardware();
        waitForStart();

        while(opModeIsActive()){
            int rPos = 0, lPos = 0;
            rPos += gamepad2.right_stick_x/50;
            lPos += gamepad2.left_stick_x/50;

            rPos = Range.clip(rPos, 0, 1);
            lPos = Range.clip(lPos, 0, 1);

            //right.setPosition(1-(rPos*.077));
            //left.setPosition(.923+(lPos*.077));

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            double rotatedX = (Math.sqrt(2) / 2) * (x + y);
            double rotatedY = (Math.sqrt(2) / 2) * (-x + y);

            double x2 = Math.pow(rotatedX, 2);
            double y2 = Math.pow(rotatedY, 2);

            double cts = makeANumber(Math.sqrt((x2 + y2) / Math.max(x2, y2)));

            double xScaled = rotatedX * cts;
            double yScaled = rotatedY * cts;
            double turn = gamepad1.right_stick_x;

            a.setPower(((yScaled)) + (turn));
            b.setPower(((-xScaled)) + (turn));
            c.setPower(((xScaled)) + (turn));
            d.setPower(((-yScaled)) + (turn));

            if(gamepad1.right_bumper) {
                sweep.setPower(1);
            }
            if(gamepad1.left_bumper) {
                sweep.setPower(-1);
            }
            else {
                sweep.setPower(0);
            }
            if (gamepad1.right_trigger > .75){
                flick.setPower(-1);
            }
            else {
                flick.setPower(0);
            }

            if(gamepad1.x){
                //left.setDirection(Servo.Direction.FORWARD);
                //right.setDirection(Servo.Direction.REVERSE);
                left.setPosition(1);
            }
            else{
                //left.setDirection(Servo.Direction.REVERSE);
                left.setPosition(.85);
                //right.setDirection(Servo.Direction.FORWARD);
            }
            if(gamepad1.b){
                right.setPosition(.85);
            }
            else{
                right.setPosition(1);
            }
            telemetry.addData("Left", left.getPosition());
            telemetry.addData("Right", right.getPosition());
            telemetry.update();
        }
    }
}