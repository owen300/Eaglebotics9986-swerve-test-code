package org.firstinspires.ftc.teamcode.motorcontrol;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class dualmotor {
    DcMotorEx m;
    DcMotorEx m1;
    PIDCoefficientsEx pid;
    PIDEx pidc;
    PIDFCoefficients pidf;
    PIDFController pidfc;
    public void dualmotor(String m,String m1,HardwareMap hardwareMap){
        this.m=hardwareMap.get(DcMotorEx.class, m);
        this.m1=hardwareMap.get(DcMotorEx.class, m1);

    }
    public void setPower(double p){
        m.setPower(p);
        m1.setPower(p);
    }

    public void setPID(double p, double i, double d){
        pid = new PIDCoefficientsEx(p,i,d,1,0.5,0.5);
        pidc= new PIDEx(pid);

    }

    public void runPID(double target,double current){
        m.setPower(pidc.calculate(target, current));
        m1.setPower(pidc.calculate(target, current));
    }
    public void setPIDF(double p, double i, double d, double f, double t){
        pidfc= new PIDFController(p,i,d,f);
        pidfc.setTolerance(t);
    }
    public void runPIDF(double target,double current){
        m.setPower(pidfc.calculate(target, current));
        m1.setPower(pidfc.calculate(target, current));
    }


}
