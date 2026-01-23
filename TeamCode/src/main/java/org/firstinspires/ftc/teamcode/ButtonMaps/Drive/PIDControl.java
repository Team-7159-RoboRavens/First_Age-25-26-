package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.pedropathing.util.Timer;
public class PIDControl {
    private double kp;
    private double ki;
    private double kd;
    private double prevErr;
    private double err;
    private Timer dt;
    private double integral;

    private double time;
    public PIDControl(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
        prevErr = 0;
        err = 0;
        dt = new Timer();
        integral = 0;
    }
    public double output() {
        return kp * err + ki * integral + kd * (err-prevErr)/time;

    }
    public void update(double newErr) {
        time = dt.getElapsedTimeSeconds();
        dt = new Timer();
        prevErr = err;
        err = newErr;
        integral += err * time;
    }
}
