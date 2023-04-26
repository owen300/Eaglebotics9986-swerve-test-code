package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class testthreading extends Thread{
    public double x=0;
    public double y=0;
    public testthreading(Gamepad gamepad){
        x=x;
        y= y;
    }
    //@Override
    public void run(Gamepad gamepad){
        x=gamepad.left_stick_x;
        y=-gamepad.left_stick_y;

    }
}
