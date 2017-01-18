package org.firstinspires.ftc.teamcode.org.suffernrobotics.Autonomous;
//Contains the values(1 or -1) to set the motor direction, such that the robot moves in a direction
public enum MechanumDirection {FORWARD(1,1,-1,-1), RIGHT(1,-1,1,-1), LEFT(-1,1,-1,1),BACKWARD(-1,-1,1,1),ROTATE(1,1,1,1);
    public final int fl;
    public final int fr;
    public final int bl;
    public final int br;
    MechanumDirection(int fl,int fr,int bl,int br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }
}
