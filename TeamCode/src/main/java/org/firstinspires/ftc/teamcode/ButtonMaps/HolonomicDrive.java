package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HolonomicDrive {
    //thanks game manual 0!
    //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    public static MotorPowers fieldOrientedDrive(Gamepad gamepad, double maxMotorPower, IMU imu){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //Provide a deadzone of +-0.1
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rotate = gamepad.right_stick_x > 0.1 || gamepad.right_stick_x < -0.1 ? gamepad.right_stick_x : 0;
        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
        double frontLeftPower = maxMotorPower*((rotY + rotX + rotate) / denominator);
        double backLeftPower = maxMotorPower*((rotY - rotX + rotate) / denominator);
        double frontRightPower = maxMotorPower*((rotY - rotX - rotate) / denominator);
        double backRightPower = maxMotorPower*((rotY + rotX - rotate) / denominator);
        frontLeftPower *= .5; backLeftPower *= .5; frontRightPower *= .5; backRightPower *= .5;
        return new MotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public static MotorPowers JoystickHoloDrive(Gamepad gamepad1, OpMode opMode) {
        // Read joystick values
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        opMode.telemetry.addData("HoloDrive y", y);
        opMode.telemetry.addData("HoloDrive x", x);
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);

        double frontLeftPower = (y + x) / denominator;
        double backLeftPower = (y - x) / denominator;
        double frontRightPower = (y - x) / denominator;
        double backRightPower = (y + x) / denominator;
        opMode.telemetry.addData("HoloDrive frontLeft", frontLeftPower);
        return new MotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public MotorPowers fieldOrientedDrive(double x, double y, double turn, double maxMotorPower, double robotAngle) {
        double rotX = x * Math.cos(robotAngle) - y * Math.sin(robotAngle);
        double rotY = x * Math.sin(robotAngle) + y * Math.cos(robotAngle);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = maxMotorPower * (rotY + rotX - turn) / denominator;
        double backLeftPower = maxMotorPower * (rotY - rotX - turn) / denominator;
        double frontRightPower = maxMotorPower * (rotY - rotX + turn) / denominator;
        double backRightPower = maxMotorPower * (rotY + rotX + turn) / denominator;
        return new MotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public MotorPowers robotOrientedDrive(double x, double y, double turn, double maxMotorPower){
        return fieldOrientedDrive(x, y, turn, maxMotorPower, 0);
    }
}
