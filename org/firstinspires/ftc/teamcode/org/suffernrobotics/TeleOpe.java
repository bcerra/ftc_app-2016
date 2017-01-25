package org.firstinspires.ftc.teamcode.org.suffernrobotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpe", group = "OpModes")
public class TeleOpe extends LinearOpMode {

    DcMotor a, b, c, d, sweep, flick, lift;
    Servo left, right;

    ColorSensor c1;
    GyroSensor g1;
    OpticalDistanceSensor o1;

    public void initHardware() {
        //Initialize servos
        left = hardwareMap.servo.get("l");
        right = hardwareMap.servo.get("r");

        //Initialize motors
        a = hardwareMap.dcMotor.get("a");
        b = hardwareMap.dcMotor.get("b");
        c = hardwareMap.dcMotor.get("c");
        d = hardwareMap.dcMotor.get("d");
        sweep = hardwareMap.dcMotor.get("s");
        flick = hardwareMap.dcMotor.get("f");
        //lift = hardwareMap.dcMotor.get("ll");

        c1 = hardwareMap.colorSensor.get("c1");
        g1 = hardwareMap.gyroSensor.get("g1");
        o1 = hardwareMap.opticalDistanceSensor.get("o1");

    }

    public double makeANumber(double number) {
        if (Double.isNaN(number))
            return 0;
        return number;
    }

    @Override
    public void runOpMode(){
        initHardware();
        double rPos = .5, lPos = 1;

        waitForStart();

        while(opModeIsActive()){

            /*rPos += gamepad2.right_stick_x/50;
            lPos += gamepad2.left_stick_x/50;

            rPos = Range.clip(rPos, 0, 1);
            lPos = Range.clip(lPos, 0, 1);

            right.setPosition(1-(rPos*.077));
            left.setPosition(.923+(lPos*.077));*/

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
            } else if(gamepad1.left_trigger > .75){
                flick.setPower(1);
            }
            else {
                flick.setPower(0);
            }

            if(gamepad2.x){
                rPos = .65;
                lPos = .85;
            }
            if(gamepad2.b){
                rPos = .5;
                lPos = 1;
            }

            right.setPosition(Range.clip(rPos, 0, 1));
            left.setPosition(Range.clip(lPos, 0, 1));
            telemetry.addData("Left", left.getPosition());
            telemetry.addData("Right", right.getPosition());
            telemetry.update();
        }
    }
}