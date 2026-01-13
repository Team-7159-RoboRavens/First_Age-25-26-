package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

public class PIDControl {
    public static double p = 0;
    public static double proportional = 0;
    public static double integral = 0;
    public static double derivative = 0;
    public static double dt = 0.02;
    public static double prevErr;
    public double integral(double err) {
        integral += err*dt;
    }
    public double proportional(double err) {

    }
}
