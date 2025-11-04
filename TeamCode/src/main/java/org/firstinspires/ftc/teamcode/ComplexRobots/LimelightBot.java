package org.firstinspires.ftc.teamcode.ComplexRobots;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.limelightData;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.MecanumDriveNoMotors;

import java.util.List;

//This is right now the same as the TrikeRobot, add Pivot turn at some point and some more functionality.

@Config
public class LimelightBot extends MecanumDriveNoMotors {
    enum Direction {
        UP,DOWN
    }
    OpMode opMode;
    //    public final DcMotorEx ShootMotor1;
    //    public final DcMotorEx ShootMotor2;
    //    public final Servo aimServo;
    //    public final Servo angleServo;
    //    public final Servo intakeServo;


//    public final Servo turnServo;
    public final Limelight3A limelight;

    public LimelightBot(HardwareMap hardwareMap, Pose2d pose, OpMode opMode) {
//        super(hardwareMap, pose);
        this.opMode = opMode;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


//        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        opMode.telemetry.addData(">", "Robot Ready.  Press Play.");
        opMode.telemetry.update();
        limelightData.accurate = false;

        //Initialize Servos
//        turnServo = hardwareMap.get(Servo.class, "turnServo");
//        aimServo = hardwareMap.get(Servo.class, "aimServo");
//        angleServo = hardwareMap.get(Servo.class, "angleServo");
//        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        //Initialize Motors
//        ShootMotor1 = hardwareMap.get(DcMotorEx.class, "ShootMotor1");
//        ShootMotor2 = hardwareMap.get(DcMotorEx.class, "ShootMotor2");



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

    public void runLimelight(int id){

            LLStatus status = limelight.getStatus();
            opMode.telemetry.addData("Name", "%s",
                    status.getName());
            opMode.telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            opMode.telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
//                opMode.telemetry.addData("LL Latency", captureLatency + targetingLatency);
//                opMode.telemetry.addData("Parse Latency", parseLatency);
//                opMode.telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                opMode.telemetry.addLine("Limelight Works!");

                if (limelightData.accurate)
                    opMode.telemetry.addLine("Accurate Data!");
                else
                    opMode.telemetry.addLine("No Accuracy!");


//
                if (result.isValid()) {
//                    opMode.telemetry.addData("tx", result.getTx());
//                    opMode.telemetry.addData("txnc", result.getTxNC());
//                    opMode.telemetry.addData("ty", result.getTy());
//                    opMode.telemetry.addData("tync", result.getTyNC());

                    opMode.telemetry.addData("Botpose", botpose.toString());

                    // Access fiducial results (April Tags)
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    if (fiducialResults.isEmpty()) {
                        //This makes sure that if there are no detected april tags, it will not take old data
                        limelightData.accurate = false;
                        opMode.telemetry.addLine("Nothing detected!");
                    }
                    boolean temp = false;
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {

//                        opMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        if (fr.getFiducialId() == id) {
                            temp = true;
                            limelightData.setParams(fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                            limelightData.accurate = true;
                            opMode.telemetry.addData("Correct tag: ", fr.getFiducialId());
                            opMode.telemetry.addData("X: ", fr.getTargetXDegrees());
                            opMode.telemetry.addData("y              opMode.telemetry.addData(\"X: \", fr.getTargetXDegrees());: ", fr.getTargetYDegrees());
                            double targetOffsetAngle_Vertical = fr.getTargetYDegrees();

                            // how many degrees back is your limelight rotated from perfectly vertical? (To be Measured.
                            double limelightMountAngleDegrees = 0;

                            // distance from the center of the Limelight lens to the floor (To be Measured)
                            double limelightLensHeightCm = 20.0;

                            // distance from the target to the floor
                            double goalHeightCm = 47;

                            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                            //calculate distance
                            double distanceFromLimelightToGoalCm = (goalHeightCm - limelightLensHeightCm) / Math.tan(angleToGoalRadians);
                            limelightData.distance = distanceFromLimelightToGoalCm;
                            opMode.telemetry.addData("Distance: ", distanceFromLimelightToGoalCm);



                        }

                    }
                    if (!temp) {
                        limelightData.accurate = false;
                        opMode.telemetry.addLine("Not right tag!");

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
                }
                else{
                    limelightData.accurate = false;
                    opMode.telemetry.addLine("No Tag!");

                }

            } else {
                opMode.telemetry.addData("Limelight", "No data available");
                //Makes sure that we are only using data that is exists at the right moment, not old data or missing data.
                limelightData.accurate = false;
            }

            opMode.telemetry.update();
    }


    public void setServosTo(double min, double max, double value, Servo servo) {
        double scaledVal = (value - min) / (max - min);
        servo.setPosition(scaledVal);
    }
}

