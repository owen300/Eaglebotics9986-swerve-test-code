/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Event;
import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.StateTransition;
import org.firstinspires.ftc.teamcode.Subsystems.teledrive.mecdt;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="thing")

public class newthing extends LinearOpMode {

    // Declare OpMode members.



    @Override
    public void runOpMode() {
        StateMachine state = new StateMachine();
        Event gp =new Event() {
            @Override
            public String getName() {
                return null;
            }
        };
        State down = new State() {
            @Override
            public void onEnter(Event event) {

            }

            @Override
            public void onExit(Event event) {

            }
        };
        State up = new State() {
            @Override
            public void onEnter(Event event) {

            }

            @Override
            public void onExit(Event event) {

            }
        };
        StateTransition goup = new StateTransition(down,gp,up);
        StateTransition godown=new StateTransition(up,gp,down);
        state.addTransition(goup);
        state.addTransition(godown);
        mecdt dt = new mecdt();
        dt.init(hardwareMap,0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        boolean reset = false;


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double x=0;
        double rx;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a){
                x=1;
            }if(gamepad1.b){
                x=0;
            }
            if(x==1){rx=gamepad1.right_stick_x*0.7;} else{rx=gamepad1.right_stick_x;}
            if(gamepad1.left_bumper){
                dt.run(gamepad1.left_stick_y,gamepad1.left_stick_x,rx,0.5,true);
            }else {
                dt.run(gamepad1.left_stick_y, gamepad1.left_stick_x, rx, 1, true, gamepad1.back);
            }
            // Setup a variable for each drive wheel to save power level for telemetry

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
}
