package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.ShootingFunctions;
import org.firstinspires.ftc.teamcode.limelightData;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;

@Config
public class FirstAgeArm extends ServoAbstractButtonMap {
    //TODO: Change back to private final when done with dash
    private MotorPowers mp;// = new MotorPowers(0);
    private double servoPosition;
    private double stage = 0;
    private double timeSince;
    private double timeBuffer = 2000;

    //These magic numbers are not final and should be iteratively tested.
    public static double baseShotPower = .40;
    public static double limelightPowerMultiplier = 1.18;
    public static double limelightBaseDistance = 100;
    public static double nonLinearPower = 1.0028;
    public static double shootVel;
    public static double targetVel;
    //This stores the motor power of the velocity at the target velocity.

    public static double onSpeed = 0;

    @Override
    public void loop(ServoTempBot robot, OpMode opMode) {

        shootVel = robot.ShootMotor.getVelocity();
        targetVel = -FirstAgeArm.velocityShot(limelightData.distance)* .9;
        opMode.telemetry.addData("Velocity ", shootVel);
        opMode.telemetry.addData("target velocity = ", targetVel);


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
            robot.Servo1.setPower(-.8);
            opMode.telemetry.addLine("Servos forward");
        }
        else if (opMode.gamepad2.b) {
            robot.Servo1.setPower(.8);
            opMode.telemetry.addLine("Servos Back");
        }
        else {
            robot.Servo1.setPower(0);
        }

        if (opMode.gamepad2.y) {
            robot.Servo2.setPosition(-.3);
            opMode.telemetry.addLine("Servos forward");
        }



        if (opMode.gamepad2.dpad_down) {
            if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
                opMode.telemetry.addLine("Shoot medium-short");
                robot.ShootMotor.setPower(baseShotPower * -1.45);
            }
            else {
                opMode.telemetry.addLine("Shoot Short");
                ShootingFunctions.setVelocityReworked(-700, shootVel ,robot.ShootMotor,-1);
            }
        }
        else if (opMode.gamepad2.dpad_up) {

            if (stage == 0) {
                timeSince = System.currentTimeMillis();
//                robot.Servo3.setPower(0);
                robot.Servo2.setPosition(.2);
            }
            stage = 1;
            if (timeSince + 4200 < System.currentTimeMillis() && timeSince + 6000 > System.currentTimeMillis()) {
                robot.Servo2.setPosition(.2);
            }
            else if (timeSince + timeBuffer < System.currentTimeMillis()) {
                robot.Servo1.setPower(-.8);
//                robot.Servo3.setPower(.5);
                robot.Servo2.setPosition(-.3);
                opMode.telemetry.addLine("Servos");
            }
            opMode.telemetry.addLine("Shoot limelight");
            //This is meant to shoot according to the distance to the april tag if the limelight is accurate            //All of these variables are yet to be tested and should be iterated on
//            robot.ShootMotor.setPower(limelightData.accurate ? limelightPowerMultiplier * Math.pow(nonLinearPower, limelightData.distance) * baseShotPower : baseShotPower * 1.5);
            if (limelightData.accurate) {
                robot.ShootMotor.setPower((targetVel - shootVel) / 100);
            }
            else if (!limelightData.accurate) {
                opMode.telemetry.addLine("Shoot far");
                ShootingFunctions.setVelocityReworked(1700, shootVel, robot.ShootMotor,-1);
            }
        }
        else if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
            if (opMode.gamepad2.dpad_up) {
                opMode.telemetry.addLine("Shoot medium-long");
                robot.ShootMotor.setPower(baseShotPower * -1.55);
            }
            else {
                opMode.telemetry.addLine("Shoot medium");
                robot.ShootMotor.setVelocity(-700);
            }
        }
        else {
            robot.ShootMotor.setPower(0);
            stage = 0;
//            robot.Servo3.setPower(0);
            robot.Servo2.setPosition(.2);
        }


        //Aim manually with the left joystick
        if (Math.abs(opMode.gamepad2.left_stick_y) > 0.2 || Math.abs(opMode.gamepad2.left_stick_x) > 0.2) {
            //                robot.setServosTo(-1, 1, opMode.gamepad2.left_stick_x, robot.aimServo);
            //                robot.setServosTo(-1, 1, opMode.gamepad2.left_stick_y, robot.angleServo);
            opMode.telemetry.addLine("Aiming manually, x/y: " + opMode.gamepad2.left_stick_x + opMode.gamepad2.left_stick_y);
        }






    }

    public static double velocityShot(double x) {
        return (2.07096 * Math.pow(10, -16) * .3 * Math.pow(x, 2) + 7.81571 * x + 550.14286);
    }
}
