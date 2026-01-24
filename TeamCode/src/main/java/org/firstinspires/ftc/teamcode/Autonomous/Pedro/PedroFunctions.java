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
        opMode.telemetry.addData("Velocity ", shootVel);
        targetVel = velocityShot(limelightData.distance);
        robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        opMode.telemetry.addLine("Shoot limelight");
        //This is meant to shoot according to the distance to the april tag if the limelight is accurate            //All of these variables are yet to be tested and should be iterated on
//            robot.ShootMotor.setPower(limelightData.accurate ? limelightPowerMultiplier * Math.pow(nonLinearPower, limelightData.distance) * baseShotPower : baseShotPower * 1.5);
        if (limelightData.accurate) {
            robot.intakeMotor1.setPower(1);
            robot.intakeMotor2.setPower(1);
            robot.ShootMotor.setPower((targetVel - shootVel) / 137);
            opMode.telemetry.addLine("Limelight passes in data");
        } else {
            opMode.telemetry.addLine("Shoot far");
            opMode.telemetry.addLine("Limelight not working");
            robot.intakeMotor1.setPower(1);
            robot.intakeMotor2.setPower(.6);
            robot.ShootMotor.setPower((velocityShot(185) - shootVel) / 137);
        }
    }

    public static void aim(ServoGoodBot robot) {


        if (Math.abs(limelightData.aprilXDegrees / 20) < aimingThreshold && limelightData.accurate) {
            limelightData.aiming = false;
            opMode.telemetry.addLine("Aimed");
            opMode.telemetry.addData("value is:", String.valueOf(Math.abs(limelightData.aprilXDegrees / 400)));
            robot.setMotorPowers(new MotorPowers(0, 0, 0, 0));
        } else if ((Math.abs(limelightData.aprilXDegrees / 20) >= aimingThreshold) && limelightData.accurate) {
            limelightData.aiming = true;
            opMode.telemetry.addLine("Aiming");
            robot.leftFront.setPower(-pid.output());
            robot.leftBack.setPower(-pid.output());
            robot.rightFront.setPower(pid.output());
            robot.rightBack.setPower(pid.output());
            limelightData.aiming = false;
            opMode.telemetry.addData("value is:", String.valueOf(Math.abs(limelightData.aprilXDegrees / 400)));
        }
    }


    public static void intake(ServoGoodBot robot) {
        robot.intakeMotor2.setPower(1);
        robot.intakeMotor1.setPower(-1);
    }
}