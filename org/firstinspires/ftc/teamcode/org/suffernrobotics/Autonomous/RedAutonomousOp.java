package org.firstinspires.ftc.teamcode.org.suffernrobotics.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//By extending my AutonomousOp class and overriding the team method, the code is the same for both
//teams, but is automatically adjusted for each side, because of the difference in the team method
@Autonomous(name = "Autonomous Red", group = "Autonomous")
class RedAutonomousOp extends AutonomousOp{
    @Override
    public ColorType team(){
        return ColorType.RED;
    }
}
