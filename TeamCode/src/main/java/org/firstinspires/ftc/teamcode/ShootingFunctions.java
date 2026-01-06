package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShootingFunctions {

    private static final double learningRate = .03;
    public static double currentPower = .7;

    //This function returns the velocity in which the shooting motor should spin to shoot a given distance.
    public static double velocityShot(double x) {
        return (2.07096 * Math.pow(10, -16) * 1 * Math.pow(x, 2) +  9.28571 * x + 500.14286);
    }

    //This function is called in a loop to minimize the distance between a target velocity and the current velocity of a given motor,
    public static void setVelocity(double targetVel, double currentVel, DcMotorEx motor, double muliplier) {
        motor.setVelocity(targetVel * muliplier);
    }

    //This funnction is similar to the previous function but it should be better.
    // This is for wheels that brake like driving ones so they can reach their target velocity slower and not overshoot as much.
    public static void setVelocityReworked(double targetVel, double currentVel, DcMotorEx motor, int sign) {

        targetVel = Math.abs(targetVel);
        currentVel = Math.abs(currentVel);

        if ((targetVel - currentVel) < (targetVel / 300)) {
            currentPower -=  learningRate * (currentVel / (targetVel + Math.pow(10,-8)));
        } else if ((targetVel - currentVel) > (targetVel / 300)) {
            currentPower += learningRate * (targetVel / (currentVel + Math.pow(10,-8)));
        }
        if (currentPower > 1) {
            currentPower = 1;
        } else if (currentPower < 0) {
            currentPower = 0;
        }

        motor.setPower(currentPower * sign);
    }
}
