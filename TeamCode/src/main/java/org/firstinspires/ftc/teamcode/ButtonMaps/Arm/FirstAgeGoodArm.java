package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.limelightData;
import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

public class FirstAgeGoodArm extends ServoAbstractButtonMapGood{
    //TODO: Change back to private final when done with dash
    private MotorPowers mp;// = new MotorPowers(0);
    private double servoPosition;
    private double stage = 0;
    private double timeSince;
    private double timeBuffer = 3000;

    //These magic numbers are not final and should be iteratively tested.
    public static double baseShotPower = .40;
    public static double limelightPowerMultiplier = 1.18;
    public static double limelightBaseDistance = 100;
    public static double nonLinearPower = 1.0028;
    public static double shootVel;
    public static double targetVel;

    @Override
    public void loop(ServoGoodBot robot, OpMode opMode) {

//        if (opMode.gamepad2.options) {
//            robot.intakeMotor.setPower(.8);
//        }

        shootVel = robot.ShootMotor.getVelocity();
        opMode.telemetry.addData("Velocity ", shootVel);
        targetVel = velocityShot(limelightData.distance);

        //Automatically Aim if there is a tag
        if (opMode.gamepad2.x) {
            if (limelightData.accurate) {
                timeSince = opMode.getRuntime();
                limelightData.aiming = true;
//                robot.setServosTo(-1, 1, limelightData.directionToTag()[0], robot.aimServo);
//                robot.setServosTo(-1, 1, limelightData.directionToTag()[1], robot.angleServo);
                opMode.telemetry.addLine("Aiming");
            }
            else{
                opMode.telemetry.addLine("No Tag");
                limelightData.aiming = false;
            }
        }
//        if (Math.abs(opMode.gamepad2.left_stick_y) > .2) {
//            robot.Servo1.setPower(opMode.gamepad2.left_stick_y);
//            robot.Servo2.setPower(-opMode.gamepad2.left_stick_y);
//            opMode.telemetry.addData("Servos Going", opMode.gamepad2.left_stick_y);
//        }
//        else{
//            robot.Servo1.setPower(0);
//            robot.Servo2.setPower(0);
//        }

        if (opMode.gamepad2.a) {
            robot.intakeMotor.setPower(.8);
        }
        else {
            robot.intakeMotor.setPower(0);
        }
        if (opMode.gamepad2.b) {
            robot.Servo1.setPosition(.8);
            opMode.telemetry.addLine("Servos Back");
        }
        else if (opMode.gamepad2.x) {
            robot.Servo1.setPosition(-.8);
            opMode.telemetry.addLine("Servos  x        else {
//            robot.Servo1.setPower(0);
//        }

//        if (opMode.gamepad2.y) {
//            robot.Servo2.setPosition(.4);
//            opMode.telemetry.addLine("Servos forward");
//        }



        if (opMode.gamepad2.dpad_down) {
            if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
                opMode.telemetry.addLine("Shoot medium-short");
                robot.ShootMotor.setPower(baseShotPower * 1.45);
            }
            else {
                opMode.telemetry.addLine("Shoot Short");
                robot.ShootMotor.setPower(baseShotPower * 1.35);
            }
        }
        else if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
            if (opMode.gamepad2.dpad_up) {
                opMode.telemetry.addLine("Shoot medium-long");
                robot.ShootMotor.setPower(baseShotPower * 1.55);
            }
            else {
                opMode.telemetry.addLine("Shoot medium");
                robot.ShootMotor.setPower(baseShotPower * 1.5);
            }
        }
        else if (opMode.gamepad2.dpad_up) {

//            if (stage == 0) {
//                timeSince = System.currentTimeMillis();
//                robot.Servo3.setPower(0);
//                robot.Servo2.setPosition(.7);
//            }
//            stage = 1;
//            if (timeSince + 4600 < System.currentTimeMillis() && timeSince + 6500 > System.currentTimeMillis()) {
//                robot.Servo2.setPosition(.7);
//                robot.Servo1.setPower(-.1);
//            }
//            else if (timeSince + timeBuffer < System.currentTimeMillis()) {
//                robot.Servo1.setPower(-.8);
//                robot.Servo3.setPower(.5);
//                robot.Servo2.setPosition(.4);
//                opMode.telemetry.addLine("Servos");
//            }
            opMode.telemetry.addLine("Shoot limelight");
            //This is meant to shoot according to the distance to the april tag if the limelight is accurate
            //All of these variables are yet to be tested and should be iterated on
//            robot.ShootMotor.setPower(limelightData.accurate ? limelightPowerMultiplier * Math.pow(nonLinearPower, limelightData.distance) * baseShotPower : baseShotPower * 1.5);
            if (limelightData.accurate)
                robot.ShootMotor.setPower((targetVel - shootVel) / 60);
            if (!limelightData.accurate)
                opMode.telemetry.addLine("Shoot far");

        }
        else {
            robot.ShootMotor.setPower(0);
            stage = 0;
//            robot.Servo3.setPower(0);
//            robot.Servo2.setPosition(.7);
        }


        //Aim manually with the left joystick
        if (Math.abs(opMode.gamepad2.left_stick_y) > 0.2 || Math.abs(opMode.gamepad2.left_stick_x) > 0.2) {
            //                robot.setServosTo(-1, 1, opMode.gamepad2.left_stick_x, robot.aimServo);
            //                robot.setServosTo(-1, 1, opMode.gamepad2.left_stick_y, robot.angleServo);
            opMode.telemetry.addLine("Aiming manually, x/y: " + opMode.gamepad2.left_stick_x + opMode.gamepad2.left_stick_y);
        }






    }

    public static double velocityShot(double x) {
        return (2.07096 * Math.pow(10, -16) * 0.97 * Math.pow(x, 2) +  9.28571 * x + 481.14286);
    }
}
