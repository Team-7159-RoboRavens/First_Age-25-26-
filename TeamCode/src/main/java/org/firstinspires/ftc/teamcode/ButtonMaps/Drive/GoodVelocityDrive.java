package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

//import com.acmerobotics.dashboard.config.Config;

import static org.firstinspires.ftc.teamcode.ButtonMaps.DPadControl.dpadStrafe;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ButtonMaps.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ShootingFunctions;
import org.firstinspires.ftc.teamcode.limelightData;

//@Config
public class GoodVelocityDrive extends ServoAbstractButtonMapGood {
    // defines deadzones for triggers and joystick
    //MAGIC NUMBERS!!!!!
static double triggerDeadZone = .1;
static double triggerLinearity = 1; //1 is linear relation, 2 is quadratic finer controll at lower motor speeds less at high speeds, .2 is opposite controll at high speeds
static double joystickDeadZone = .1;
static double joystickLinearity = 3;

static double aimingPower = 1;
static double aimingThreshold = .045;

static double startTime;
static private boolean motorBrake = true;

private static ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void loop(ServoGoodBot robot, OpMode opMode) {

        MotorPowers mp;
        mp = getMotorPowers(
                robot,
                robot.lazyImu.get(),
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

        //Reset FOD
        if (opMode.gamepad1.back) {
            robot.lazyImu.get().resetYaw();
        }


        if (opMode.gamepad2.x){
            if (Math.abs(limelightData.aprilXDegrees / 20) < aimingThreshold) {
                limelightData.aiming = false;
                opMode.telemetry.addLine("Aimed");
            }
            else if (limelightData.accurate) {
                limelightData.aiming = true;
                opMode.telemetry.addLine("Aiming");
                mp.leftFront += limelightData.aprilXDegrees / 20 * Math.pow(limelightData.aprilXDegrees, -.3) * aimingPower + .1;
                mp.leftBack += limelightData.aprilXDegrees / 20  * Math.pow(limelightData.aprilXDegrees, -.3) * aimingPower + .1;
                mp.rightFront -= limelightData.aprilXDegrees / 20 * Math.pow(limelightData.aprilXDegrees, -.3)  * aimingPower + .1;
                mp.rightBack -= limelightData.aprilXDegrees / 20 * Math.pow(limelightData.aprilXDegrees, -.3) * aimingPower + .1;
//                smallTimedPedro.rotate(limelightData.aprilXDegrees + 4, robot);
                limelightData.aiming = false;
                opMode.telemetry.addLine("Aimed");
            }
//            else
//                opMode.telemetry.addLine("Can't Aim");
        }

        mp.leftFront += dpadStrafe(opMode, .8).leftFront;
        mp.rightFront += dpadStrafe(opMode, .8).rightFront;
        mp.leftBack += dpadStrafe(opMode, .8).leftBack;
        mp.rightBack += dpadStrafe(opMode, .8).rightBack;

        Motors.setMotorPower(mp.leftFront, robot.leftFront, robot.leftFront.getVelocity());
        Motors.setMotorPower(mp.rightFront, robot.rightFront, robot.rightFront.getVelocity());
        Motors.setMotorPower(mp.leftBack, robot.leftBack, robot.leftBack.getVelocity());
        Motors.setMotorPower(mp.rightBack, robot.rightBack, robot.rightBack.getVelocity());

//        robot.leftFront.setVelocity(mp.leftFront * 380);
//        robot.rightFront.setVelocity(mp.rightFront * 380);
//        robot.leftBack.setVelocity(mp.leftBack * 380);
//        robot.rightBack.setVelocity(mp.rightBack * 380);

        opMode.telemetry.addLine("WheelVel " + robot.leftFront.getVelocity());
        opMode.telemetry.addLine("WheelVel " + robot.rightFront.getVelocity());
        opMode.telemetry.addLine("WheelVel " + robot.leftBack.getVelocity());
        opMode.telemetry.addLine("WheelVel " + robot.rightBack.getVelocity());

        double robotHeading = -robot.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        opMode.telemetry.addLine("angle: "+robotHeading);
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
            double turnSpeed = -Math.pow((right_stick_x-triggerDeadZone), triggerLinearity)/Math.pow((1-triggerDeadZone), triggerLinearity);
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

        robot.lazyImu.get();
        opMode.telemetry.addLine("forward: "+forward);
        opMode.telemetry.addLine("right: "+right);
        opMode.telemetry.addLine("turn: "+turn);

        opMode.telemetry.addLine("left stick x: "+left_stick_x+ "\ny: "+left_stick_y);

        double robotHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return HolonomicDrive.fieldOrientedDrive(right, forward, turn, maxMotorPower, robotHeading, opMode);

    }


}