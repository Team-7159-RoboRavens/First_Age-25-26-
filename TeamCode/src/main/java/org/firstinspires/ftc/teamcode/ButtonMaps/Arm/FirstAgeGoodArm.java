package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.Pedro.PedroFunctions;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.DualLogger;
import org.firstinspires.ftc.teamcode.ShootingFunctions;
import org.firstinspires.ftc.teamcode.limelightData;

public class FirstAgeGoodArm extends ServoAbstractButtonMapGood {
    //TODO: Change back to private final when done with dash
    private double pressVelocity;
    private double stage = 0;
    private double timeSince;
    private final double timeBuffer = 2000;
//    boolean timeDelay = false;


    //These magic numbers are not final and should be iteratively tested.
    public static double baseShotPower = .40;
    public static double shootVel;
    public static double targetVel;
    static double joystickDeadZone = .1;


    @Override
    public void loop(ServoGoodBot robot, OpMode opMode) {

        //These coefficients are used in the shooting code later.
        //    private final double timeBuffer2 = 100;
        double timeSince2 = opMode.getRuntime();
        shootVel = robot.shootMotor.getVelocity();
        robot.dualLogger.addData("Velocity ", shootVel);
        targetVel = velocityShot(limelightData.distance);
        robot.dualLogger.addData("Target Velocity ", targetVel);
        robot.shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        robot.shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.dualLogger.addData("Angle to Tag ", limelightData.aprilXDegrees);


        //This shoots short to test launcher power, they should be changed to relevent velocity shots for easier testing.
        if (opMode.gamepad2.dpad_down) {
            robot.shootMotor.setPower(-.4);
//            robot.shootMotor2.setPower(-.4);
//                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//                if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
//                    opMode.telemetry.addLine("Shoot medium-short");
////                    robot.ShootMotor.setPower(baseShotPower * 1.45);
//                    robot.intakeMotor1.setPower(.8);
//                    robot.intakeMotor2.setPower(.75);
//                } else {
//                    opMode.telemetry.addLine("Shoot Short");
////                    robot.ShootMotor.setPower(baseShotPower * 1.35);
//                    robot.intakeMotor1.setPower(0);
//                    robot.intakeMotor2.setPower(-.3);
//                }
        } else if (opMode.gamepad2.dpad_up) {
            robot.shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            robot.shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (stage == 0) {
                timeSince = System.currentTimeMillis();
                pressVelocity = limelightData.accurate ? targetVel : Math.abs(velocityShot(278));
            }
            robot.dualLogger.addData("Press Target Velocity", pressVelocity);
            stage = 1;
            if (timeSince + timeBuffer > System.currentTimeMillis()) {
//                opMode.telemetry.addLine("No Balls");
            } else {
                if (Math.abs(pressVelocity - shootVel) < 50) {
                    robot.intakeMotor1.setPower(.75);
                    robot.intakeMotor2.setPower(.8);
//                    robot.dualLogger.addData("IntakeMotor2 Velocity", robot.intakeMotor2.getVelocity());
                } else {
                    robot.intakeMotor1.setPower(0);
                    robot.intakeMotor2.setPower(-.4);
                }
            }
            if (limelightData.accurate) {
                robot.shootMotor.setVelocity(pressVelocity);
//                robot.shootMotor2.setVelocity(pressVelocity);
//                robot.dualLogger.addLine("Limelight passes in shot");
            }
            //This allows you to shoot the ball far even if the limelight disconnects or misses the tag.
            else {
//                robot.dualLogger.addLine("Shoot far");
//                robot.dualLogger.addLine("Limelight not working");
                robot.shootMotor.setVelocity(pressVelocity);
//                robot.shootMotor2.setVelocity(pressVelocity);
            }
//            opMode.telemetry.addLine("Shoot limelight");
            //This is meant to shoot according to the distance to the april tag if the limelight is accurate.
            //A PIDFF controller may want to be implemented.

            //These are just modes for testing launcher power.
        } else if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
            robot.shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            robot.shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (opMode.gamepad2.dpad_up) {
                opMode.telemetry.addLine("Shoot medium-long");
                robot.shootMotor.setPower(baseShotPower * 1.55);
//                robot.shootMotor2.setPower(baseShotPower * 1.55);
            } else {
                opMode.telemetry.addLine("Shoot medium");
                robot.shootMotor.setPower(baseShotPower * 1.5);
//                robot.shootMotor2.setPower(baseShotPower * 1.5);
            }
            //This slows the launcher down again when you stop shooting,
        } else {
            robot.shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.shootMotor.setPower(0);
//            robot.shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            robot.shootMotor2.setPower(0);
            stage = 0;
        }

        //Intake balls and shoot them into the launcher.
        if (opMode.gamepad2.left_stick_y > joystickDeadZone && opMode.gamepad2.dpad_up) {
            robot.intakeMotor1.setPower(1 * opMode.gamepad2.left_stick_y);
            robot.intakeMotor2.setPower(1 * opMode.gamepad2.left_stick_y);
        }
        //Intake balls without feeding them into the launcher.
        else if (opMode.gamepad2.left_stick_y > joystickDeadZone && !opMode.gamepad2.dpad_up) {
            robot.intakeMotor1.setPower(opMode.gamepad2.left_stick_y);
//                robot.intakeMotor2.setPower(.5);
        }
        //This is for clearing the launcher if something is stuck.
        else if (opMode.gamepad2.left_stick_y < -joystickDeadZone && !opMode.gamepad2.dpad_up) {
            robot.intakeMotor1.setPower(1 * opMode.gamepad2.left_stick_y);
            robot.intakeMotor2.setPower(1 * opMode.gamepad2.left_stick_y);
        }
        //When you don't want the first intake to move and just want to move artifacts to the launcher
        else if (opMode.gamepad2.left_stick_y < -joystickDeadZone && opMode.gamepad2.dpad_up) {
            robot.intakeMotor1.setPower(0);
            robot.intakeMotor2.setPower(1 * opMode.gamepad2.left_stick_y);
        }
        //Run both motors without having to turn on the shooting motor.
        if (opMode.gamepad2.b) {
            PedroFunctions.intake(robot);
        }

        if (opMode.gamepad2.y) {
            robot.intakeMotor2.setPower(0.65);
        }

        //The end case where none of the relevent buttons are pressed so the motors don't just keep spinning.
        else if (Math.abs(opMode.gamepad2.left_stick_y) < joystickDeadZone && !opMode.gamepad2.dpad_up) {
            robot.intakeMotor1.setPower(0);
            robot.intakeMotor2.setPower(0);
        }
    }

    public static double velocityShot(double x) {
        //Old
//        return (2.07096 * Math.pow(10, -16) * .3 * Math.pow(x, 2) + 7.81571 * x + 470.14286);
        return 2.84926 * x + 1235.65423;
    }
}
