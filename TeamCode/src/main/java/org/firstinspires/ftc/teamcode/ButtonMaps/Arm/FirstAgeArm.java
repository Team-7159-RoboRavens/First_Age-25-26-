package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.limelightData;
import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@Config
public class FirstAgeArm extends ServoAbstractButtonMap {
    //TODO: Change back to private final when done with dash
    private MotorPowers mp;// = new MotorPowers(0);
    private double servoPosition;
    private double timeSince;
    private double timeBuffer = 200;

    //These magic numbers are not final and should be iteratively tested.
    private double baseShotPower = .5;
    private double limelightPowerMultiplier =.5;
    private double limelightBaseDistance = 100;

    @Override
    public void loop(ServoTempBot robot, OpMode opMode) {

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
            robot.Servo1.setPower(-.6);
            opMode.telemetry.addLine("Servos");
        }
        else {
            robot.Servo1.setPower(0);
        }

        if (opMode.gamepad2.dpad_down) {
            if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
                opMode.telemetry.addLine("Shoot medium-short");
                robot.ShootMotor.setPower(baseShotPower * 1.15);
            }
            else {
                opMode.telemetry.addLine("Shoot Short");
                robot.ShootMotor.setPower(baseShotPower);
            }
        }
        else if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
            if (opMode.gamepad2.dpad_up) {
                opMode.telemetry.addLine("Shoot medium-long");
                robot.ShootMotor.setPower(baseShotPower * 1.55);
            }
            else {
                opMode.telemetry.addLine("Shoot medium");
                robot.ShootMotor.setPower(baseShotPower * 1.3);
            }
        }
        else if (opMode.gamepad2.dpad_up) {
            robot.Servo1.setPower(-.6);
            opMode.telemetry.addLine("Servos");
            opMode.telemetry.addLine("Shoot limelight");
            //This is meant to shoot according to the distance to the april tag if the limelight is accurate
            //All of these variables are yet to be tested and should be iterated on
            robot.ShootMotor.setPower(limelightData.accurate ? limelightPowerMultiplier * Math.pow((limelightData.distance - limelightBaseDistance), .8) / limelightBaseDistance + (1.4 * baseShotPower) : baseShotPower * 1.5);
            if (!limelightData.accurate)
                opMode.telemetry.addLine("Shoot far");

        }
        else {
            robot.ShootMotor.setPower(0);
        }


        //Aim manually with the left joystick
        if (Math.abs(opMode.gamepad2.left_stick_y) > 0.2 || Math.abs(opMode.gamepad2.left_stick_x) > 0.2) {
            //                robot.setServosTo(-1, 1, opMode.gamepad2.left_stick_x, robot.aimServo);
            //                robot.setServosTo(-1, 1, opMode.gamepad2.left_stick_y, robot.angleServo);
            opMode.telemetry.addLine("Aiming manually, x/y: " + opMode.gamepad2.left_stick_x + opMode.gamepad2.left_stick_y);
        }






    }
}
