package org.firstinspires.ftc.teamcode.motorcontrol;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class dualmotor {
    DcMotor m;
    DcMotor m1;
    PIDCoefficientsEx pid;
    PIDEx pidc;
    PIDFCoefficients pidf;
    PIDFController pidfc;

    public void dualmotorinit(String m,String m1,HardwareMap hardwareMap){
        this.m=hardwareMap.dcMotor.get(m);
        this.m1=hardwareMap.dcMotor.get(m1);
        this.m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPower(double p){
        m.setPower(p);
        m1.setPower(p);
    }

    public void setPID(double p, double i, double d){
        pid = new PIDCoefficientsEx(p,i,d,1,0.5,0.5);
        pidc= new PIDEx(pid);

    }

    public void runPID(double target){
        m.setPower(pidc.calculate(target, m.getCurrentPosition()));
        m1.setPower(m.getPower());
    }
    public void setPIDF(double p, double i, double d, double f, double t){
        pidfc= new PIDFController(p,i,d,f);
        pidfc.setTolerance(t);
    }
    public void runPIDF(double target){
        pidfc.setSetPoint(target);
        m.setPower(pidfc.calculate(m.getCurrentPosition()));
        m1.setPower(m.getPower());
    }


}
