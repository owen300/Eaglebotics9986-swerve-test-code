/* Copyright (c) 2021 FIRST. All rights reserved.
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



public class testrfclass{
    public static double P = 0.45, I = 0.01, D = 0.1;
    public static double K_STATIC = 0.04;
    private PIDFController rotationController;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor encoder = null;
   private CRServo servo;
   public double axonpower=0;
    double currentpos = 0;//encoder.getCurrentPosition();
    double currentangle=0;
    double axial   = 0;  // Note: pushing stick forward gives negative value
    double lateral =  0;
    double target= 0;
    public static double MAX_SERVO = 1, MAX_MOTOR = 1;
    public double error;
    public double power1 = 0;
    public double power;
    public double i=0;
    public testrfclass(Gamepad gamepad1, HardwareMap hardwareMap) {
        rotationController = new PIDFController(P, I, D, 0);

        rotationController.setTolerance(1);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
//        encoder=hardwareMap.dcMotor.get("enc");
//        servo=hardwareMap.crservo.get("test");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.


        // Wait for the game to start (driver presses PLAY)


        runtime.reset();
    }
        // run until the end of the match (driver presses STOP)
        public void run(Gamepad gamepad) {
            runtime.reset();
            rotationController.setPIDF(P, I, D, 0);
            double max;
            currentpos = 0;//encoder.getCurrentPosition();
            currentangle=normalizeDegrees((currentpos/5)*360);
            axial   = -gamepad.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral =  gamepad.left_stick_x;
             target= Math.toDegrees(Math.atan2(lateral,axial));
            if(axial == 0&&lateral==0)target=0;
            power=Math.sqrt((axial*axial)+(lateral*lateral));
            power=((power/1.189)*1);
//            if (((target>=0&&target<=45)||(target<0&&target>=-45))&&gamepad.right_stick_x>0){
//                power= Range.clip(power*-gamepad.right_stick_x,0, 1);
//            } else if ((target>45&&target<=135)&&gamepad.right_stick_x<0){
//                power= Range.clip(power*gamepad.right_stick_x,0, 1);
//            }else if (((target>=135&&target<=180)||(target<-135&&target>=-180))&&gamepad.right_stick_x<0) {
//                power= Range.clip(power*gamepad.right_stick_x,0, 1);
//            }else if ((target>-135&&target<=-45)&&gamepad.right_stick_x>0){
//                power= Range.clip(power*-gamepad.right_stick_x,0, 1);
//            }
            if (((target>=0&&target<=45)||(target<0&&target>=-45))&&gamepad.right_stick_x>0){
            power= Range.clip(power/gamepad.right_stick_x,0, 1);
            } else if ((target>45&&target<=135)&&gamepad.right_stick_x<0){
            power= Range.clip(power/-gamepad.right_stick_x,0, 1);
            }else if (((target>=135&&target<=180)||(target<-135&&target>=-180))&&gamepad.right_stick_x<0) {
            power= Range.clip(power/-gamepad.right_stick_x,0, 1);
            }else if ((target>-135&&target<=-45)&&gamepad.right_stick_x>0){
            power= Range.clip(power/gamepad.right_stick_x,0, 1);
            }

            if(axial==0 && lateral==0 &&gamepad.right_stick_x<0){
                if(Math.abs(135-currentangle)<=180){
                    target= 135;
                    power=gamepad.right_stick_x;
                } else if(Math.abs(135-currentangle)>180){
                    target= -45;
                    power=(-gamepad.right_stick_x);
                }
            }else if(axial==0 && lateral==0&&gamepad.right_stick_x>0){
                if(Math.abs(135-currentangle)<=180){
                    target= 135;
                    power=gamepad.right_stick_x;
                } else if(Math.abs(135-currentangle)>180){
                    target= -45;
                    power=(-gamepad.right_stick_x);
                }
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            error = normalizeDegrees(target - currentangle);
            if (Double.isNaN(error)) error = 0;

            power1 = Range.clip(rotationController.calculate(0, error), -MAX_SERVO, MAX_SERVO);
            if (Double.isNaN(power1)) power1 = 0;
            //servo.setPower(power1 + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power1));
            axonpower=power1 + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power1);
//            if((gamepad1.right_stick_x<=0||gamepad1.right_stick_x>=0)&&(power==0)){
//                if (gamepad1.right_stick_x<0){
//
//                }
//            }
          //  double angle=Math.atan2(axial,lateral);
//            if (Math.abs(angle-currentangle)>180){
//                target=angle-180;
//                power=-1*power;
//            }else{
//                target=angle;
//            }
//            if(target<0){
//                target=-1*target;
//                target=target+180
//            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.

            //runtime.reset();
        }
    }

