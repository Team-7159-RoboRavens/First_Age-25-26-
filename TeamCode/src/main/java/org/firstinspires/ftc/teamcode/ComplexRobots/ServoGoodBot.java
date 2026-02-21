package org.firstinspires.ftc.teamcode.ComplexRobots;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FlywheelPDIFF;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.DualLogger;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.limelightData;

import java.util.List;

//This is right now the same as the TrikeRobot, add Pivot turn at some point and some more functionality.

@Config
public class ServoGoodBot extends MecanumDrive {
    enum Direction {
        UP, DOWN
    }

    OpMode opMode;
    public final DualLogger dualLogger;
    public final DcMotorEx ShootMotor;
    public final DcMotorEx intakeMotor1;
    public final DcMotorEx intakeMotor2;
    public GoBildaPinpointDriver pinpoint;
    //    public final DcMotorEx ShootMotor2;
//        public final Servo Servo1;
//        public final Servo Servo1;
//        public final CRServo Servo3;
//        public final DcMotorEx intakeMotor;


    //    public final Servo turnServo;
    public final Limelight3A limelight;

    public void setPinpoint(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    public ServoGoodBot(HardwareMap hardwareMap, Pose2d pose, OpMode opMode, DualLogger dualLogger) {
        super(hardwareMap, pose);
        this.dualLogger = dualLogger;
        this.opMode = opMode;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


//        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        dualLogger.addLine("Robot Ready. Press Play.");
        opMode.telemetry.update();
        limelightData.accurate = false;

        //Initialize Servos
//        Servo1 = hardwareMap.get(Servo.class, "servo1");
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
//        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FlywheelPDIFF.P, 0, 0, FlywheelPDIFF.F));
//        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ShootMotor2 = hardwareMap.get(DcMotorEx.class, "ShootMotor2");
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intakeMotor1");
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intakeMotor2");
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set Imu

        if (!limelightData.hasImu) {
            lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        } else {
            lazyImu = limelightData.imu;
        }
        limelightData.setIMU(lazyImu);


        //Initialize Output Servo
//        turnServo.scaleRange(-1,1);
//        aimServo.scaleRange(-1,0);
//        intakeServo.scaleRange(-1,0);
//        angleServo.scaleRange(-1,0);
    }


    public MotorPowers setAllMotorPowers(int i) {
        return new MotorPowers(0, 0, 0, 0);
    }


    public void setMotorTo(DcMotorEx motor, int targetPos, double power) {
        if (motor.getCurrentPosition() < targetPos) {
            while (motor.getCurrentPosition() <= targetPos) {
                motor.setPower(power);
            }
        } else if (motor.getCurrentPosition() > targetPos) {
            while (motor.getCurrentPosition() >= targetPos) {
                motor.setPower(-power);
            }
        }
    }

    public static double yOffset(double x) {
        return (0.00000149143 * Math.pow(x, 4) - 0.0000406469 * Math.pow(x, 3) + 0.00156737 * Math.pow(x, 2) - 0.0680153 * x);
    }

    public void runLimelight(int id) {

        LLStatus status = limelight.getStatus();
        dualLogger.addData("Name", "%s",
                status.getName());
        dualLogger.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        dualLogger.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            dualLogger.addData("LL Latency", captureLatency + targetingLatency);
            dualLogger.addData("Parse Latency", parseLatency);
            dualLogger.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
            dualLogger.addLine("Limelight Works!");

            if (result.isValid()) {

                dualLogger.addData("tx", result.getTx());
                dualLogger.addData("txnc", result.getTxNC());
                dualLogger.addData("ty", result.getTy());
                dualLogger.addData("tync", result.getTyNC());

                dualLogger.addData("Botpose", botpose.toString());
                if (limelightData.accurate) {
                    dualLogger.addLine("Correct: ");
//                    dualLogger.addData("Aiming ", limelightData.aiming);
                }
//                else
//                    dualLogger.addLine("Bad");

                // Access fiducial results (April Tags)
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (fiducialResults.isEmpty()) {
                    //This makes sure that if there are no detected april tags, it will not take old data
                    limelightData.accurate = false;
                }
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    dualLogger.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    if (fr.getFiducialId() == id) {
                        limelightData.setParams(fr.getFiducialId(), fr.getFamily(), id == 24 ? fr.getTargetXDegrees() + 1.4 : fr.getTargetXDegrees() - 2.2, fr.getTargetYDegrees() - ServoGoodBot.yOffset(fr.getTargetXDegrees()));
                        limelightData.accurate = true;
                        dualLogger.addData("Correct tag: ", fr.getFiducialId());
                        dualLogger.addData("X: ", fr.getTargetXDegrees());
                        dualLogger.addData("y              ", fr.getTargetYDegrees());
//                        dualLogger.addData("\"X: \"", fr.getTargetXDegrees());
                        dualLogger.addData("Direction to Tag", limelightData.aprilXDegrees);
//                        dualLogger.addData("Robot Pose Field Space",fr.getRobotPoseFieldSpace());
//                        dualLogger.addData("Robot Pose Target Space",fr.getRobotPoseTargetSpace());
//                        dualLogger.addData("Target Pose Robot Space",fr.getTargetPoseRobotSpace());




                        double targetOffsetAngle_Vertical = fr.getTargetYDegrees();

                        // how many degrees back is your limelight rotated from perfectly vertical? (To be Measured.
                        double limelightMountAngleDegrees = (id == 20 ? 2.5 : 2.5);

                        // distance from the center of the Limelight lens to the floor (To be Measured)
                        double limelightLensHeightCm = 28.0;

                        // distance from the target to the floor
                        double goalHeightCm = 75;

                        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                        //calculate distance
                        double distanceFromLimelightToGoalCm = (goalHeightCm - limelightLensHeightCm) / Math.tan(angleToGoalRadians);
                        limelightData.distance = distanceFromLimelightToGoalCm;
                        dualLogger.addData("Distance: ", distanceFromLimelightToGoalCm);
                    }
                    if (fr.getFiducialId() > 20 && fr.getFiducialId() < 24) {
                        limelightData.pattern = fr.getFiducialId();
                        dualLogger.addData("Pattern, ", fr.getFiducialId());
                    }
                }

                // Access color results
//                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//                    int temp = 0;
//                    LLResultTypes.ColorResult colorResult = colorResults.get(0);
//                    for (LLResultTypes.ColorResult cr : colorResults) {
//                        if (colorResult.getTargetArea() < colorResults.get(temp).getTargetArea())
//                            temp++;
//                        else
//                            colorResult = colorResults.get(temp);
//                    }
//                    if (colorResult.getTargetXPixels() > 120)
//                        telemetry.addData("Largest Yellow Object", String.valueOf(colorResult.getTargetXDegrees()), String.valueOf(colorResult.getTargetYDegrees()));
//
            } else {
                limelightData.accurate = false;
            }
        } else {
            dualLogger.addData("Limelight", "No data available");
            //Makes sure that we are only using data that is exists at the right moment, not old data or missing data.
            limelightData.accurate = false;
        }
        dualLogger.addLine("________________________________________________");

//            opMode.telemetry.update();
    }


//    public void setServosTo(double min, double max, double value, Servo servo) {
//        double scaledVal = (value - min) / (max - min);
//        servo.setPosition(scaledVal);
//    }
}

