package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelPDIFF extends OpMode {
    public DcMotorEx flywheel;

    double F = 0;

    double P = 0;

    public double highVelocity = 1500;

    public  double lowVelocity = 900;

    public double curTargetVelocity = highVelocity;

    double[] stepSizes = {10.0, 1.0 , 0.1, .001, .0001};

    int stepIndex = 1;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "shootMotor");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        //Normal opmode code

        if (gamepad2.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
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
        //Set new coeficients

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        flywheel.setVelocity(curTargetVelocity);

        double curVelocity = flywheel.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("F", F);
        telemetry.addData("P", P);
        telemetry.addData("Current Velocity", curVelocity);
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Error", error);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Tuning P", P);
        telemetry.addData("Tuning F", F);
        telemetry.update();

    }

}
