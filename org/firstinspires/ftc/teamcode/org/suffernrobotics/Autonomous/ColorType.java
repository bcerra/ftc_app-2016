package org.firstinspires.ftc.teamcode.org.suffernrobotics.Autonomous;

//Red and Blue have an integer value for their team,
//because the ColorType enum is used for both to represent the color "seen" by the color sensor
//and the color of the team
public enum ColorType {UNDEFINED(0), BLUE(1), RED(2);
    public final int value;
    ColorType(int value) {
        this.value = value;
    }
    public int getDirection(){
        return (int) ((-1.5*Math.pow(this.value,2))+(2.5*this.value));//Returns 0 if undefined, 1 if blue and -1 if red.
    }
}
