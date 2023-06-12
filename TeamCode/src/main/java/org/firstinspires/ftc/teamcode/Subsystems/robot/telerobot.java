package org.firstinspires.ftc.teamcode.Subsystems.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.teledrive.mecdt;

public class telerobot {
    private double x =0;
    private mecdt tdt = new mecdt();
    public void init(HardwareMap hardwareMap, double rc1){
        tdt.init(hardwareMap,rc1);

    }
    public void run(Gamepad gamepad1,GamepadEx gp){
        double rx;
        if(gamepad1.a){
            x=1;
        }if(gamepad1.b){
            x=0;
        }
        if(x==1){rx=gamepad1.right_stick_x*0.7;} else{rx=gamepad1.right_stick_x;}
        if(gamepad1.left_bumper){
            tdt.run(gamepad1.left_stick_y,gamepad1.left_stick_x,rx,0.5,true);
        }else {
            tdt.run(gamepad1.left_stick_y, gamepad1.left_stick_x, rx, 1, true, gamepad1.back);
        }

    }
}
