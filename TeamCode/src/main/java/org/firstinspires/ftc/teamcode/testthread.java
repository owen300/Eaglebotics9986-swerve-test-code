package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class testthread extends LinearOpMode {
    testthreading thread = new testthreading(gamepad1);
    double x=0;
    double y=0;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while(opModeIsActive()) {

            thread.start();
            while(thread.isAlive()){
                x=thread.x;
                y= thread.y;
                if (gamepad1.left_bumper){
                    x=0;
                }
                telemetry.addData("x",x);
                telemetry.addData("y",y);
            }
        }
    }
}
