package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

//import com.acmerobotics.dashboard.config.Config;

import static org.firstinspires.ftc.teamcode.ButtonMaps.DPadControl.dpadStrafe;
import static org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDrive.pid;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ButtonMaps.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
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
static double aimingThreshold = .02;

static private boolean motorBrake = true;

private static ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static PIDControl pid = new PIDControl(0.036, 0, 0);


    @Override
    public void loop(ServoGoodBot robot, OpMode opMode) {
        limelightData.ImuOffset = 0;
        IMU imu = robot.lazyImu.get();
        // FOD resetting
        if (opMode.gamepad1.back && et.time() > 500) {
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
                opMode);





        if (opMode.gamepad2.x){
            if (Math.abs(limelightData.aprilXDegrees / 20) < aimingThreshold && limelightData.accurate) {
                limelightData.aiming = false;
                opMode.telemetry.addLine("Aimed");
                opMode.telemetry.addData("value is:", String.valueOf(Math.abs(limelightData.aprilXDegrees / 20)));
                mp = new MotorPowers(0, 0, 0, 0);
                robot.setMotorPowers(mp);
            }
            else if ((Math.abs(limelightData.aprilXDegrees / 20) >= aimingThreshold) && limelightData.accurate) {
                limelightData.aiming = true;
                mp.leftFront += pid.output();
                mp.leftBack += pid.output();
                mp.rightFront -= pid.output();
                mp.rightBack -= pid.output();
                limelightData.aiming = false;
                opMode.telemetry.addData("turning value", pid.output());
                opMode.telemetry.addData("value is:", String.valueOf(Math.abs(limelightData.aprilXDegrees / 20)));
            }
        }
        pid.update(limelightData.aprilXDegrees);

        mp.leftFront += dpadStrafe(opMode, .8).leftFront;
        mp.rightFront += dpadStrafe(opMode, .8).rightFront;
        mp.leftBack += dpadStrafe(opMode, .8).leftBack;
        mp.rightBack += dpadStrafe(opMode, .8).rightBack;

        robot.setMotorPowers(new MotorPowers(-mp.leftFront, -mp.rightFront, -mp.leftBack, -mp.rightBack));
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        opMode.telemetry.addLine("angle: "+robotHeading);
//        opMode.telemetry.addLine("Pinpoint Heading: "+ robot.pinpoint.getHeading(AngleUnit.RADIANS));
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
            OpMode opMode) {
        double right = 0;
        double forward = 0;
        double turn = 0;
        double maxMotorPower = 1;

        //Turn Left or Right
        if (left_trigger > triggerDeadZone || right_trigger > triggerDeadZone) {
            double turnSpeed = Math.pow((right_trigger-triggerDeadZone), triggerLinearity)/Math.pow((1-triggerDeadZone), triggerLinearity) - Math.pow((left_trigger-triggerDeadZone), triggerLinearity)/Math.pow((1-triggerDeadZone), triggerLinearity); //look mommy an afront to coders everywhere (it also works first try :333)
            turn += turnSpeed;
        }

        if (Math.abs(right_stick_x) > joystickDeadZone) {
            double turnSpeed = Math.pow((right_stick_x-triggerDeadZone), triggerLinearity)/Math.pow((1-triggerDeadZone), triggerLinearity);
            turn += turnSpeed;
        }

        //When left bumper is pressed, go backward
        if (left_bumper) {
            forward --;
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
            double turnSpeed = Math.pow((right_trigger-triggerDeadZone), triggerLinearity)/Math.pow((1-triggerDeadZone), triggerLinearity) - Math.pow((left_trigger-triggerDeadZone), triggerLinearity)/Math.pow((1-triggerDeadZone), triggerLinearity);
            double forwardSpeed = 0;
            double strafeSpeed = 0;
            if (left_stick_y > joystickDeadZone) {
                forwardSpeed = Math.pow((left_stick_y-joystickDeadZone), joystickLinearity)/Math.pow((1-joystickDeadZone), joystickLinearity);
            }
            if (left_stick_x > joystickDeadZone) {
                strafeSpeed = Math.pow((left_stick_x-joystickDeadZone), joystickLinearity)/Math.pow((1-joystickDeadZone), joystickLinearity);
            }
            if (left_stick_y < -joystickDeadZone) {
                forwardSpeed = -Math.abs(Math.pow((left_stick_y+joystickDeadZone), joystickLinearity)/Math.pow((1-joystickDeadZone), joystickLinearity)); //congrats you got the the end of this line of code, would you like to see more :3
            }
            if (left_stick_x < -joystickDeadZone) {
                strafeSpeed = -Math.abs(Math.pow((left_stick_x + joystickDeadZone), joystickLinearity) / Math.pow((1 - joystickDeadZone), joystickLinearity));
            }
            forward -= forwardSpeed;
            right += strafeSpeed;
            turn = turnSpeed;
        }

        //Slow strafe while holding x
        if (x) {
            maxMotorPower *= 0.5;
        }

        //Toggle whether the robot is in brake or coast mode
        if(opMode.gamepad1.b && et.time() > 500){
            et.reset();
            if(motorBrake){
                robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorBrake = false;
            }else{
                robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBrake = true;
            }
        }

        if(motorBrake){
            opMode.telemetry.addData("Drive Motor Mode", "Brake");
        }else{
            opMode.telemetry.addData("Drive Motor Mode", "Coast");
        }
        opMode.telemetry.addLine("forward: "+forward);
        opMode.telemetry.addLine("right: "+right);
        opMode.telemetry.addLine("turn: "+turn);

        opMode.telemetry.addLine("left stick x: "+left_stick_x+ "\ny: "+left_stick_y);

        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return HolonomicDrive.fieldOrientedDrive(right, forward, turn, maxMotorPower, robotHeading, opMode);

    }
}