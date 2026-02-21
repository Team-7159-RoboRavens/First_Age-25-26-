package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

//import com.acmerobotics.dashboard.config.Config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.ButtonMaps.DPadControl.dpadStrafe;
import static org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDrive.pid;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.TimeBased.TimeAutoFunctions;
import org.firstinspires.ftc.teamcode.ButtonMaps.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.DualLogger;
import org.firstinspires.ftc.teamcode.limelightData;

//@Config
public class LiamPolarDriveGood extends ServoAbstractButtonMapGood {
    // defines deadzones for triggers and joystick
    //MAGIC NUMBERS!!!!!
    static double triggerDeadZone = .1;
    static double triggerLinearity = 1; //1 is linear relation, 2 is quadratic finer controll at lower motor speeds less at high speeds, .2 is opposite controll at high speeds
    static double joystickDeadZone = .1;
    static double joystickLinearity = 3;

    static double aimingPower = 1;
    static double aimingThreshold = .004;

    static private boolean motorBrake = true;
    private Pose2D position;

    private static ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static PIDControl pid = new PIDControl(0.04, 0, 0);


    @Override
    public void loop(ServoGoodBot robot, OpMode opMode) {
        position = robot.pinpoint.getPosition();
        limelightData.ImuOffset = 0;
        IMU imu = robot.lazyImu.get();
        // FOD resetting
        if (opMode.gamepad1.back && et.time() > 500) {
            robot.pinpoint.resetPosAndIMU();
            imu.resetYaw();
            et.reset();
        }
        MotorPowers mp;
        mp = getMotorPowers(
                robot,
                imu,
                opMode.gamepad1.dpad_up,
                opMode.gamepad1.dpad_down,
                opMode.gamepad1.dpad_left,
                opMode.gamepad1.dpad_right,
                opMode.gamepad1.left_bumper,
                opMode.gamepad1.right_bumper,
                opMode.gamepad1.left_trigger,
                opMode.gamepad1.right_trigger,
                opMode.gamepad1.left_stick_y,
                opMode.gamepad1.left_stick_x,
                opMode.gamepad1.right_stick_x,
                opMode.gamepad1.x,
                opMode, position);


        if (opMode.gamepad2.x) {
            if (Math.abs(limelightData.aprilXDegrees / 20) < aimingThreshold && limelightData.accurate) {
                limelightData.aiming = false;
                opMode.telemetry.addLine("Aimed");
                robot.dualLogger.addData("value is:", String.valueOf(Math.abs(limelightData.aprilXDegrees / 20)));
                mp = new MotorPowers(0, 0, 0, 0);
                robot.setMotorPowers(mp);
            } else if ((Math.abs(limelightData.aprilXDegrees / 20) >= aimingThreshold) && limelightData.accurate) {
                limelightData.aiming = true;
                mp.leftFront += pid.output();
                mp.leftBack += pid.output();
                mp.rightFront -= pid.output();
                mp.rightBack -= pid.output();
                limelightData.aiming = false;
                robot.dualLogger.addData("turning value", pid.output());
                robot.dualLogger.addData("value is:", String.valueOf(Math.abs(limelightData.aprilXDegrees / 20)));
            }
        }
        if (opMode.gamepad2.a && limelightData.accurate) {
            mp.leftFront += TimeAutoFunctions.aim(limelightData.aprilXDegrees, 0, .6, opMode.telemetry, robot).leftFront;
            mp.leftBack += TimeAutoFunctions.aim(limelightData.aprilXDegrees, 0, .6, opMode.telemetry, robot).rightFront;
            mp.rightFront -= TimeAutoFunctions.aim(limelightData.aprilXDegrees, 0, .6, opMode.telemetry, robot).leftBack;
            mp.rightBack -= TimeAutoFunctions.aim(limelightData.aprilXDegrees, 0, .6, opMode.telemetry, robot).rightBack;
        }
        pid.update(limelightData.aprilXDegrees);

        mp.leftFront += dpadStrafe(opMode, .8).leftFront;
        mp.rightFront += dpadStrafe(opMode, .8).rightFront;
        mp.leftBack += dpadStrafe(opMode, .8).leftBack;
        mp.rightBack += dpadStrafe(opMode, .8).rightBack;

        robot.setMotorPowers(new MotorPowers(-mp.leftFront, -mp.rightFront, -mp.leftBack, -mp.rightBack));
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        opMode.telemetry.addLine("angle: " + robotHeading);
        robot.dualLogger.addData("Angle", robotHeading);
        robot.dualLogger.addLine("Pinpoint Heading: " + position.getHeading(AngleUnit.RADIANS));
        robot.dualLogger.addData("Pinpoint X", position.getX(DistanceUnit.CM));
        robot.dualLogger.addData("Pinpoint Y", position.getY(DistanceUnit.CM));
    }

    public static MotorPowers getMotorPowers(
            ServoGoodBot robot,
            IMU imu,
            boolean dpad_up,
            boolean dpad_down,
            boolean dpad_left,
            boolean dpad_right,
            boolean left_bumper,
            boolean right_bumper,
            double left_trigger,
            double right_trigger,
            double left_stick_y,
            double left_stick_x,
            double right_stick_x,
            boolean x,
            OpMode opMode,
            Pose2D position) {
        double right = 0;
        double forward = 0;
        double turn = 0;
        double maxMotorPower = 1;


        if (Math.abs(right_stick_x) > joystickDeadZone) {
            double turnSpeed = Math.pow((right_stick_x - triggerDeadZone), triggerLinearity) / Math.pow((1 - triggerDeadZone), triggerLinearity);
            robot.dualLogger.addData("RightStick Pressed", turnSpeed);
            turn += turnSpeed;
        }
        robot.dualLogger.addData("Line 144 Turn", turn);

        //When left bumper is pressed, go backward
        if (left_bumper) {
            forward--;
        }
        //When right bumper is pressed move forward
        if (right_bumper) {
            forward++;
        }
        //the MOAMF (Mother Of All Movement Functions
        //Allows Joystick and triggers to control where the robot goes

        // Scales speed so that after DeadZone, it is increasing at a exponential
        // ,rate, so when the joystick is fully pressed the speed is 1
        if (Math.abs(left_stick_y) > joystickDeadZone || Math.abs(left_stick_x) > joystickDeadZone || left_trigger > triggerDeadZone || right_trigger > triggerDeadZone) {
//            double angle = Math.atan2(left_stick_y, left_stick_x);
//            double scalingFactor = Math.max(1, Math.abs(left_stick_x * 1.1));
            double turnSpeed = Math.pow((right_trigger - triggerDeadZone), triggerLinearity) / Math.pow((1 - triggerDeadZone), triggerLinearity) - Math.pow((left_trigger - triggerDeadZone), triggerLinearity) / Math.pow((1 - triggerDeadZone), triggerLinearity);
            double forwardSpeed = 0;
            double strafeSpeed = 0;
            if (left_stick_y > joystickDeadZone) {
                forwardSpeed = Math.pow((left_stick_y - joystickDeadZone), joystickLinearity) / Math.pow((1 - joystickDeadZone), joystickLinearity);
            }
            if (left_stick_x > joystickDeadZone) {
                strafeSpeed = Math.pow((left_stick_x - joystickDeadZone), joystickLinearity) / Math.pow((1 - joystickDeadZone), joystickLinearity);
            }
            if (left_stick_y < -joystickDeadZone) {
                forwardSpeed = -Math.abs(Math.pow((left_stick_y + joystickDeadZone), joystickLinearity) / Math.pow((1 - joystickDeadZone), joystickLinearity)); //congrats you got the the end of this line of code, would you like to see more :3
            }
            if (left_stick_x < -joystickDeadZone) {
                strafeSpeed = -Math.abs(Math.pow((left_stick_x + joystickDeadZone), joystickLinearity) / Math.pow((1 - joystickDeadZone), joystickLinearity));
            }
            forward -= forwardSpeed;
            right += strafeSpeed;
            turn += turnSpeed;
        }
        robot.dualLogger.addData("line 181 turn", turn);

        //Slow strafe while holding x
        if (x) {
            maxMotorPower *= 0.5;
        }

        //Toggle whether the robot is in brake or coast mode
        if (opMode.gamepad1.b && et.time() > 500) {
            et.reset();
            if (motorBrake) {
                robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorBrake = false;
            } else {
                robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBrake = true;
            }
        }

        if (motorBrake) {
            robot.dualLogger.addData("Drive Motor Mode", "Brake");
        } else {
            robot.dualLogger.addData("Drive Motor Mode", "Coast");
        }
        robot.dualLogger.addData("forward: ", forward);
        robot.dualLogger.addData("right: ", right);
        robot.dualLogger.addData("turn: ", turn);

        robot.dualLogger.addLine("left stick x: " + left_stick_x);
        robot.dualLogger.addLine("left stick y: " + left_stick_y);
        robot.dualLogger.addLine("right stick x: " + right_stick_x);

        double robotHeading = position.getHeading(AngleUnit.RADIANS);
        robot.dualLogger.addLine("FOD: " + robotHeading);
        return HolonomicDrive.fieldOrientedDrive(right, forward, turn, maxMotorPower, robotHeading, opMode);

    }
}