package org.firstinspires.ftc.teamcode.ComplexRobots;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FlywheelPDIFF;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MecanumDriveNoMotors;
import org.firstinspires.ftc.teamcode.limelightData;

import java.util.List;

//This is right now the same as the TrikeRobot, add Pivot turn at some point and some more functionality.

@Config
public class ShootOnlyBot extends MecanumDriveNoMotors {
    enum Direction {
        UP, DOWN
    }

    OpMode opMode;
    public final DcMotorEx ShootMotor;
    public final DcMotorEx ShootMotor2;

    public ShootOnlyBot(HardwareMap hardwareMap, Pose2d pose, OpMode opMode) {
        this.opMode = opMode;

        opMode.telemetry.addData(">", "Robot Ready.  Press Play.");
        opMode.telemetry.update();

        //Initialize Servos
//        Servo1 = hardwareMap.get(CRServo.class, "servo1");
//        Servo1 = hardwareMap.get(Servo.class, "servo1");
//        Servo3 = hardwareMap.get(CRServo.class, "servo3");
//        angleServo = hardwareMap.get(Servo.class, "angleServo");
//        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        //Initialize Motors
        ShootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Reset the motor encoder so that it reads zero ticks
        ShootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FlywheelPDIFF.P - 60, 0, 0, 1));

//        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        ShootMotor2 = hardwareMap.get(DcMotorEx.class, "shootMotor2");
        ShootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShootMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Reset the motor encoder so that it reads zero ticks
        ShootMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FlywheelPDIFF.P - 60, 0, 0, 1));

//        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER


        //Initialize Output Servo
//        turnServo.scaleRange(-1,1);
//        aimServo.scaleRange(-1,0);
//        intakeServo.scaleRange(-1,0);
//        angleServo.scaleRange(-1,0);
    }


    public MotorPowers setAllMotorPowers(int i) {
        return new MotorPowers(0,0,0,0);
    }


    public void setMotorTo(DcMotorEx motor, int targetPos, double power) {
        if (motor.getCurrentPosition() < targetPos) {
            while (motor.getCurrentPosition() <= targetPos) {
                motor.setPower(power);
            }
        }
        else if (motor.getCurrentPosition() > targetPos) {
            while (motor.getCurrentPosition() >= targetPos) {
                motor.setPower(-power);
            }
        }
    }

//    public static double yOffset(double x){
//        return (0.00000149143 * Math.pow(x, 4) - 0.0000406469 * Math.pow(x, 3) + 0.00156737 * Math.pow(x, 2) - 0.0680153 * x);
//    }

//    public void runLimelight(int id){
//
//            LLStatus status = limelight.getStatus();
//            opMode.telemetry.addData("Name", "%s",
//                    status.getName());
//            opMode.telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            opMode.telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());
//
//            LLResult result = limelight.getLatestResult();
//            if (result != null) {
//                // Access general information
//                Pose3D botpose = result.getBotpose();
//                double captureLatency = result.getCaptureLatency();
//                double targetingLatency = result.getTargetingLatency();
//                double parseLatency = result.getParseLatency();
//                opMode.telemetry.addData("LL Latency", captureLatency + targetingLatency);
//                opMode.telemetry.addData("Parse Latency", parseLatency);
//                opMode.telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
//                opMode.telemetry.addLine("Limelight Works!");
//
//                if (result.isValid()) {
//
//                    opMode.telemetry.addData("tx", result.getTx());
//                    opMode.telemetry.addData("txnc", result.getTxNC());
//                    opMode.telemetry.addData("ty", result.getTy());
//                    opMode.telemetry.addData("tync", result.getTyNC());
//
//                    opMode.telemetry.addData("Botpose", botpose.toString());
//                    if (limelightData.accurate) {
//                        opMode.telemetry.addLine("Correct: ");
//                        opMode.telemetry.addData("Aiming ", limelightData.aiming);
//                    }
//                    else
//                        opMode.telemetry.addLine("Bad");
//
//                    // Access fiducial results (April Tags)
//                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                    if (fiducialResults.isEmpty()) {
//                        //This makes sure that if there are no detected april tags, it will not take old data
//                        limelightData.accurate = false;
//                    }
//                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                        opMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                        if (fr.getFiducialId() == id) {
//                        limelightData.setParams(fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees() + limelightData.distance / 22, fr.getTargetYDegrees() - ShootOnlyBot.yOffset(fr.getTargetXDegrees()));
//                            limelightData.accurate = true;
//                            opMode.telemetry.addData("Correct tag: ", fr.getFiducialId());
//                            opMode.telemetry.addData("X: ", fr.getTargetXDegrees());
//                            opMode.telemetry.addData("y              ", fr.getTargetYDegrees() - ShootOnlyBot.yOffset(fr.getTargetXDegrees()));
//                            opMode.telemetry.addData("\"X: \"", fr.getTargetXDegrees());
//                            opMode.telemetry.addData("Direction to Tag", limelightData.directionToTag());
//
//
//
//                            double targetOffsetAngle_Vertical = fr.getTargetYDegrees() - ShootOnlyBot.yOffset(fr.getTargetXDegrees());
//
//                            // how many degrees back is your limelight rotated from perfectly vertical? (To be Measured.
//                            double limelightMountAngleDegrees = 6.5;
//
//                            // distance from the center of the Limelight lens to the floor (To be Measured)
//                            double limelightLensHeightCm = 28.0;
//
//                            // distance from the target to the floor
//                            double goalHeightCm = 75;
//
//                            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
//                            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
//
//                            //calculate distance
//                            double distanceFromLimelightToGoalCm = (goalHeightCm - limelightLensHeightCm) / Math.tan(angleToGoalRadians);
//                            limelightData.distance = distanceFromLimelightToGoalCm;
//                            opMode.telemetry.addData("Distance: ", distanceFromLimelightToGoalCm);
//                        }
//                        if (fr.getFiducialId() > 20 && fr.getFiducialId() < 24) {
//                            limelightData.pattern = fr.getFiducialId();
//                            opMode.telemetry.addData("Pattern, ", fr.getFiducialId());
//                        }
//                    }
//
//                    // Access color results
////                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
////                    int temp = 0;
////                    LLResultTypes.ColorResult colorResult = colorResults.get(0);
////                    for (LLResultTypes.ColorResult cr : colorResults) {
////                        if (colorResult.getTargetArea() < colorResults.get(temp).getTargetArea())
////                            temp++;
////                        else
////                            colorResult = colorResults.get(temp);
////                    }
////                    if (colorResult.getTargetXPixels() > 120)
////                        telemetry.addData("Largest Yellow Object", String.valueOf(colorResult.getTargetXDegrees()), String.valueOf(colorResult.getTargetYDegrees()));
////
//                }
//            } else {
//                opMode.telemetry.addData("Limelight", "No data available");
//                //Makes sure that we are only using data that is exists at the right moment, not old data or missing data.
//                limelightData.accurate = false;
//            }
//
////            opMode.telemetry.update();
//    }
//
//
////    public void setServosTo(double min, double max, double value, Servo servo) {
////        double scaledVal = (value - min) / (max - min);
////        servo.setPosition(scaledVal);
////    }
}

