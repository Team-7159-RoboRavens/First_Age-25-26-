package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShootingFunctions {

    private static final double learningRate = .1;
    private static double currentPower = .8;

    //This function returns the velocity in which the shooting motor should spin to shoot a given distance.
    public static double velocityShot(double x) {
        return (2.07096 * Math.pow(10, -16) * 1 * Math.pow(x, 2) +  9.28571 * x + 500.14286);
    }

    //This function is called in a loop to minimize the distance between a target velocity and the current velocity of a given motor,
    public static void setVelocity(double targetVel, double currentVel, DcMotorEx motor) {
        if (Math.abs(targetVel - currentVel) > 80) {
            motor.setPower((targetVel - currentVel) / 80);
            currentPower = .8;
        }
        else {
            motor.setPower(currentPower);
            currentPower += (targetVel - currentVel) / 80 * learningRate;
        }
    }
}
