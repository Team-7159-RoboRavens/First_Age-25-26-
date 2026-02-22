package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.TeleOp.PIDFFTeleOp;
import org.firstinspires.ftc.teamcode.limelightData;

public class FlywheelPDIFF extends ServoAbstractButtonMapGood {

    private int stage = 1;

    public static double F = 13.75;

    public static double P = 120.7;

    public double highVelocity = 1920;

    public double lowVelocity = 1500;

    public double curTargetVelocity = highVelocity;

    double[] stepSizes = {10.0, 1.0, 0.1, .001, .0001};

    int stepIndex = 1;

    PIDFCoefficients pidfCoefficients;

    @Override
    public void loop(ServoGoodBot robot, OpMode opMode) {
        if (stage == 1) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

            robot.shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//            robot.shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        }

        //Normal opmode code


        if (opMode.gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (opMode.gamepad2.xWasPressed()) {
            curTargetVelocity += 10;
        }
        if (opMode.gamepad2.yWasPressed()) {
            curTargetVelocity -= 10;
        }
        if (opMode.gamepad2.rightBumperWasPressed()) {
            curTargetVelocity += 1000;
        }
        if (opMode.gamepad2.leftBumperWasPressed()) {
            curTargetVelocity -= 1000;
        }
        if (opMode.gamepad2.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (opMode.gamepad2.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (opMode.gamepad2.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (opMode.gamepad2.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (opMode.gamepad2.left_stick_y > .1) {
            robot.intakeMotor1.setPower(.8);
            robot.intakeMotor2.setPower(.8);
        } else {
            robot.intakeMotor1.setPower(0);
            robot.intakeMotor2.setPower(0);
        }
        //Set new coeficients

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        robot.shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        robot.shootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //set velocity
        robot.shootMotor.setVelocity(curTargetVelocity);
//        robot.shootMotor2.setVelocity(curTargetVelocity);


        double curVelocity = robot.shootMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        opMode.telemetry.addData("F", F);
        opMode.telemetry.addData("P", P);
        opMode.telemetry.addData("Current Velocity", curVelocity);
        opMode.telemetry.addData("Step Size", stepSizes[stepIndex]);
        opMode.telemetry.addData("Target Velocity", curTargetVelocity);
        opMode.telemetry.addData("Error", error);
        opMode.telemetry.addLine("-------------------------------------");
        opMode.telemetry.addData("Tuning P", P);
        opMode.telemetry.addData("Tuning F", F);

    }

}
