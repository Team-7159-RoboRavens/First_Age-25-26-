package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

public class PIDControl {
    private double kp;
    private double ki;
    private double kd;
    private double prevErr;
    private double err;
    private double dt;
    private double integral;
    public PIDControl(double p, double i, double d, double initialErr) {
        kp = p;
        ki = i;
        kd = d;
        prevErr = initialErr;
        err = initialErr;
        dt = 0.05;
        integral = 0;
    }
    public double output() {
        return kp * err + ki * integral + kd * (err-prevErr)/dt;
    }
    public void update(double newErr) {
        prevErr = err;
        err = newErr;
        integral += err * dt;
    }
}
