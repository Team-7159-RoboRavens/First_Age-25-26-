package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.Pedro.PedroFunctions;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ShootingFunctions;
import org.firstinspires.ftc.teamcode.limelightData;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;

public class FirstAgeGoodArm extends ServoAbstractButtonMapGood{
    //TODO: Change back to private final when done with dash
    private MotorPowers mp;// = new MotorPowers(0);
    private double servoPosition = 1;
    private double stage = 0;
    private double timeSince;
    private double timeBuffer = 2000;
    private double timeBuffer2 = 600;
    private double timeSince2;


    //These magic numbers are not final and should be iteratively tested.
    public static double baseShotPower = .40;
    public static double limelightPowerMultiplier = 1.18;
    public static double limelightBaseDistance = 100;
    public static double nonLinearPower = 1.0028;
    public static double shootVel;
    public static double targetVel;
    static double joystickDeadZone = .1;



    @Override
    public void loop(ServoGoodBot robot, OpMode opMode) {

        //These coefficients are used in the shooting code later.
        timeSince2 = opMode.getRuntime();
        shootVel = robot.ShootMotor.getVelocity();
        opMode.telemetry.addData("Velocity ", shootVel);
        targetVel = velocityShot(limelightData.distance);
        opMode.telemetry.addData("Target Velocity ", targetVel);


        //Automatically Aim if there is a tag, this is not currently neccessary.
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
            //This shoots short to test launcher power, they should be changed to relevent velocity shots for easier testing.
            if (opMode.gamepad2.dpad_down) {
                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
                    opMode.telemetry.addLine("Shoot medium-short");
                    robot.ShootMotor.setPower(baseShotPower * 1.45);
                } else {
                    opMode.telemetry.addLine("Shoot Short");
                    robot.ShootMotor.setPower(baseShotPower * 1.35);
                }
            } else if (opMode.gamepad2.dpad_up) {
                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                if (stage == 0) {
                    timeSince = System.currentTimeMillis();
                }
                stage = 1;
                if (timeSince + timeBuffer > System.currentTimeMillis()) {
                    opMode.telemetry.addLine("No Balls");
                }
                else {
                    if (Math.abs(targetVel - shootVel) < 80) {
                        robot.intakeMotor1.setPower(.75);
                        robot.intakeMotor2.setPower(.8);
                        opMode.telemetry.addData("IntakeMotor2 Velocity", robot.intakeMotor2.getVelocity());
//                        if (timeSince2 + timeBuffer2 > System.currentTimeMillis()) {
//                            timeSince2 = System.currentTimeMillis();
//                            if (servoPosition == 1)
//                                servoPosition *= 0;
//                            else
//                                servoPosition = 1;
//                        }
                    }
                    else {
                        robot.intakeMotor1.setPower(0);
                        robot.intakeMotor2.setPower(-.4);
                    }
                }
                if (limelightData.accurate) {
                    robot.ShootMotor.setVelocity(targetVel);
                    opMode.telemetry.addLine("Limelight passes in shot");
                }
                //This allows you to shoot the ball far even if the limelight disconnects or misses the tag.
                else {
                    opMode.telemetry.addLine("Shoot far");
                    opMode.telemetry.addLine("Limelight not working");
                    robot.ShootMotor.setVelocity(velocityShot(194));
                }
                opMode.telemetry.addLine("Shoot limelight");
                //This is meant to shoot according to the distance to the april tag if the limelight is accurate.
                //A PIDFF controller may want to be implemented.

                //These are just modes for testing launcher power.
            } else if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                if (opMode.gamepad2.dpad_up) {
                    opMode.telemetry.addLine("Shoot medium-long");
                    robot.ShootMotor.setPower(baseShotPower * 1.55);
                } else {
                    opMode.telemetry.addLine("Shoot medium");
                    robot.ShootMotor.setPower(baseShotPower * 1.5);
                }
            //This slows the launcher down again when you stop shooting,
            } else {
                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.ShootMotor.setPower(0);
                stage = 0;
            }

            //Intake balls and shoot them into the launcher.
            if (opMode.gamepad2.left_stick_y > joystickDeadZone && opMode.gamepad2.dpad_up) {
                robot.intakeMotor1.setPower(1*opMode.gamepad2.left_stick_y);
                robot.intakeMotor2.setPower(1*opMode.gamepad2.left_stick_y);
            }
            //Intake balls without feeding them into the launcher.
            else if (opMode.gamepad2.left_stick_y > joystickDeadZone && !opMode.gamepad2.dpad_up) {
                robot.intakeMotor1.setPower(opMode.gamepad2.left_stick_y);
            }
            //This is for clearing the launcher if something is stuck.
            else if (opMode.gamepad2.left_stick_y < -joystickDeadZone && !opMode.gamepad2.dpad_up) {
                robot.intakeMotor1.setPower(1*opMode.gamepad2.left_stick_y);
                robot.intakeMotor2.setPower(1*opMode.gamepad2.left_stick_y);
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
            //The end case where none of the relevent buttons are pressed so the motors don't just keep spinning.
            else if (Math.abs(opMode.gamepad2.left_stick_y) < joystickDeadZone && !opMode.gamepad2.dpad_up) {
                robot.intakeMotor1.setPower(0);
                robot.intakeMotor2.setPower(0);
            }
    }
    public static double velocityShot(double x) {
        return (2.07096 * Math.pow(10, -16) * .3 * Math.pow(x, 2) + 7.83571 * x + 421.14286);
    }
}
