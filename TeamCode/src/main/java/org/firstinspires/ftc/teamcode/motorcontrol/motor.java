package org.firstinspires.ftc.teamcode.motorcontrol;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.lang.annotation.Target;

public class motor {
    DcMotorEx m;
    PIDCoefficientsEx pid;
    PIDEx pidc;
    PIDFCoefficients pidf;
    PIDFController pidfc;
    public void motor(String m,HardwareMap hardwareMap){
        this.m=hardwareMap.get(DcMotorEx.class, m);
    }
    public void setPower(double p){
        m.setPower(p);
    }

    public void setPID(double p, double i, double d){
        pid = new PIDCoefficientsEx(p,i,d,1,0.5,0.5);
        pidc= new PIDEx(pid);

    }

    public void runPID(double target,double current){
        m.setPower(pidc.calculate(target, current));
    }
    public void setPIDF(double p, double i, double d, double f, double t){
        pidfc= new PIDFController(p,i,d,f);
        pidfc.setTolerance(t);
    }
    public void runPIDF(double target,double current){
        m.setPower(pidfc.calculate(target, current));
    }


}
