package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.NoWheelsAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ShootOnlyBot;
import org.firstinspires.ftc.teamcode.limelightData;

public class GoodArmNoWheels extends NoWheelsAbstractButtonMap {
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
    static double joystickDeadZone = .1;



    @Override
    public void loop(ShootOnlyBot robot, OpMode opMode) {

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

            if (opMode.gamepad2.dpad_down) {
                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
                    opMode.telemetry.addLine("Shoot medium-short");
                    robot.ShootMotor.setPower(baseShotPower * 1.45);
                } else {
                    opMode.telemetry.addLine("Shoot Short");
                    robot.ShootMotor.setPower(baseShotPower * 1.35);
                }
            } else if (opMode.gamepad2.dpad_right || opMode.gamepad2.dpad_left) {
                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                if (opMode.gamepad2.dpad_up) {
                    opMode.telemetry.addLine("Shoot medium-long");
                    robot.ShootMotor.setPower(baseShotPower * 1.55);
                } else {
                    opMode.telemetry.addLine("Shoot medium");
                    robot.ShootMotor.setPower(baseShotPower * 1.5);
                }
            } else if (opMode.gamepad2.dpad_up) {
                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                opMode.telemetry.addLine("Shoot limelight");
                //This is meant to shoot according to the distance to the april tag if the limelight is accurate            //All of these variables are yet to be tested and should be iterated on
//            robot.ShootMotor.setPower(limelightData.accurate ? limelightPowerMultiplier * Math.pow(nonLinearPower, limelightData.distance) * baseShotPower : baseShotPower * 1.5);
                if (limelightData.accurate) {
                    robot.intakeMotor1.setPower(1);
                    robot.intakeMotor2.setPower(.8);
                    robot.ShootMotor.setPower((targetVel - shootVel) / 137);
                    opMode.telemetry.addLine("Limelight passes in data");
                }
                else {
                    opMode.telemetry.addLine("Shoot far");
                    opMode.telemetry.addLine("Limelight not working");
                    robot.intakeMotor1.setPower(1);
                    robot.intakeMotor2.setPower(.8);
                    robot.ShootMotor.setPower((velocityShot(185) - shootVel) / 137);
                }

            } else {
                robot.ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.ShootMotor.setPower(0);
                robot.intakeMotor1.setPower(0);
                robot.intakeMotor2.setPower(0);
                stage = 0;
//            robot.Servo3.setPower(0);
//            robot.Servo2.setPosition(.7);
            }

            //Intake Motors code
//            if (opMode.gamepad2.left_stick_y > joystickDeadZone && opMode.gamepad2.dpad_up) {
//                robot.intakeMotor1.setPower(.7);
//                robot.intakeMotor2.setPower(.4);
//            }
             if (opMode.gamepad2.left_stick_y > joystickDeadZone && !opMode.gamepad2.dpad_up) {
                robot.intakeMotor1.setPower(1);
                robot.intakeMotor2.setPower(0);
             }

             if (opMode.gamepad2.b) {
                 robot.intakeMotor1.setPower(1);
                 robot.intakeMotor2.setPower(.8);
             }
//            else if (opMode.gamepad2.left_stick_y < -joystickDeadZone && !opMode.gamepad2.dpad_up) {
//                robot.intakeMotor1.setPower(-.7);
//                robot.intakeMotor2.setPower(-.4);
//            }
//            //When you don't want the first intake to move and just want to move artifacts to the launcher
//            else if (opMode.gamepad2.left_stick_y < -joystickDeadZone && opMode.gamepad2.dpad_up) {
//                robot.intakeMotor1.setPower(0);
//                robot.intakeMotor2.setPower(.4);
//            }
//            else {
//                robot.intakeMotor1.setPower(0);
//                robot.intakeMotor2.setPower(0);
//            }
    }
    public static double velocityShot(double x) {
        return (2.07096 * Math.pow(10, -16) * .3 * Math.pow(x, 2) + 7.81571 * x + 550.14286);
    }
}
