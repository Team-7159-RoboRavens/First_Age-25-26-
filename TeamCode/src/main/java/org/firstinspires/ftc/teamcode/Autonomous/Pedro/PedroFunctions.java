package org.firstinspires.ftc.teamcode.Autonomous.Pedro;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import static org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm.velocityShot;

import static org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDriveGood.pid;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.limelightData;


public class PedroFunctions {
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
    static double joystickDeadZone = .1;
    static double aimingThreshold = .06;

    public static void shoot(ServoGoodBot robot) {

        shootVel = robot.ShootMotor.getVelocity();
        targetVel = velocityShot(limelightData.distance);
        robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //This is meant to shoot according to the distance to the april tag if the limelight is accurate            //All of these variables are yet to be tested and should be iterated on
//            robot.ShootMotor.setPower(limelightData.accurate ? limelightPowerMultiplier * Math.pow(nonLinearPower, limelightData.distance) * baseShotPower : baseShotPower * 1.5);
        if (limelightData.accurate) {
            if (Math.abs(targetVel - shootVel) < 150) {
                robot.intakeMotor1.setPower(1);
                robot.intakeMotor2.setPower(1);
            }
            robot.ShootMotor.setVelocity(velocityShot(limelightData.distance));
        } else {
            if (Math.abs(velocityShot(205) - shootVel) < 150) {
                robot.intakeMotor1.setPower(.8);
                robot.intakeMotor2.setPower(1);
            }
            else {
                robot.intakeMotor1.setPower(0);
                robot.intakeMotor2.setPower(-.4);
            }
            robot.ShootMotor.setVelocity(velocityShot(205));
        }
    }

    public static void aim(ServoGoodBot robot) {


        if (Math.abs(limelightData.aprilXDegrees / 20) < aimingThreshold && limelightData.accurate) {
            limelightData.aiming = false;
            robot.setMotorPowers(new MotorPowers(0, 0, 0, 0));
        } else if ((Math.abs(limelightData.aprilXDegrees / 20) >= aimingThreshold) && limelightData.accurate) {
            limelightData.aiming = true;
            robot.leftFront.setPower(-pid.output());
            robot.leftBack.setPower(-pid.output());
            robot.rightFront.setPower(pid.output());
            robot.rightBack.setPower(pid.output());
            limelightData.aiming = false;
        }
    }


    public static void intake(ServoGoodBot robot) {
//        robot.intakeMotor2.setPower(1);
        robot.intakeMotor1.setPower(1);
    }

    public static void reset(ServoGoodBot robot) {
        robot.intakeMotor1.setPower(0);
        robot.intakeMotor2.setPower(0);
        robot.ShootMotor.setPower(0);

    }
}