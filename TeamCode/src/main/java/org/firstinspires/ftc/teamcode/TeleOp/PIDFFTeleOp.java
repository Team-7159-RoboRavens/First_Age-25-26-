package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDriveGood;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FlywheelPDIFF;
import org.firstinspires.ftc.teamcode.ComplexRobots.ShootOnlyBot;
import org.firstinspires.ftc.teamcode.DualLogger;

@TeleOp(name = "PIDFF TeloOp")
public class PIDFFTeleOp extends OpMode {
    //Global Variables
    ShootOnlyBot robot;
    private int stage = 1;

    public static double F = 13.75;

    public static double P = 120.7;

    public double highVelocity = 1920;

    public double lowVelocity = 1500;

    public double curTargetVelocity = highVelocity;

    double[] stepSizes = {10.0, 1.0, 0.1, .001, .0001};

    int stepIndex = 1;

    PIDFCoefficients pidfCoefficients;
    //Button Maps
//    ServoAbstractButtonMapGood driveButtonMap;
//    FlywheelPDIFF armButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new ShootOnlyBot(hardwareMap, new Pose2d(0, 0, 0), this);
//        driveButtonMap = new LiamPolarDriveGood();
//        armButtonMap = new FlywheelPDIFF();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
//        driveButtonMap.loop(robot, this);
//        armButtonMap.loop(robot, this);
        if (stage == 1) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

            robot.ShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            robot.ShootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        }

        //Normal opmode code


        if (gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad2.xWasPressed()) {
            curTargetVelocity += 10;
        }
        if (gamepad2.yWasPressed()) {
            curTargetVelocity -= 10;
        }
        if (gamepad2.rightBumperWasPressed()) {
            curTargetVelocity += 1000;
        }
        if (gamepad2.leftBumperWasPressed()) {
            curTargetVelocity -= 1000;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad2.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad2.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad2.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

//        if (opMode.gamepad2.left_stick_y > .1) {
//            robot.intakeMotor1.setPower(.8);
//            robot.intakeMotor2.setPower(.8);
//        } else {
//            robot.intakeMotor1.setPower(0);
//            robot.intakeMotor2.setPower(0);
//        }
        //Set new coeficients

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        robot.ShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        robot.ShootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //set velocity
        robot.ShootMotor.setVelocity(curTargetVelocity);
        robot.ShootMotor2.setVelocity(curTargetVelocity);


        double curVelocity = robot.ShootMotor.getVelocity();
        double curVelocity2 = robot.ShootMotor2.getVelocity();

        double error = curTargetVelocity - curVelocity;
        double error2 = curTargetVelocity - curVelocity2;

        telemetry.addData("F", F);
        telemetry.addData("P", P);
        telemetry.addData("Current Velocity 1", curVelocity);
        telemetry.addData("Current Velocity 2", curVelocity2);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Error 1", error);
        telemetry.addData("Error 2", error2);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Tuning P", P);
        telemetry.addData("Tuning F", F);

//        robot.runLimelight(24);
        telemetry.update();
    }
}
