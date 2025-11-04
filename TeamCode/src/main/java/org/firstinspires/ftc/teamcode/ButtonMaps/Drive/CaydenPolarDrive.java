package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@Config
public class CaydenPolarDrive extends AbstractButtonMap {
    // defines deadzones for triggers and joystick
    //MAGIC NUMBERS!!!!!
static double triggerDeadZone = .1;
static double triggerLinearity = 1; //1 is linear relation, 2 is quadratic finer controll at lower motor speeds less at high speeds, .2 is opposite controll at high speeds
static double joystickDeadZone = .1;
static double joystickLinearity = 10;
    @Override
    public void loop(FirstAgeTempbot robot, OpMode opMode) {

        MotorPowers mp = getMotorPowers(
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
                opMode.gamepad1.x);
        opMode.telemetry.update();
        robot.setMotorPowers(mp);
    }

    public static MotorPowers getMotorPowers(
            FirstAgeTempbot robot,
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
            boolean x) {
        double right = 0;
        double forward = 0;
        double turn = 0;
        double maxMotorPower = 1;
        if (dpad_up) {
            forward ++;
        }
        if (dpad_down) {
            forward --;
        }
        if (dpad_left) {
            right --;
        }
        if (dpad_right) {
            right ++;
        }

        //Turn Left or Right
        if (left_trigger > triggerDeadZone || right_trigger > triggerDeadZone) {
            double turnSpeed = Math.pow((right_trigger-triggerDeadZone), triggerLinearity)/Math.pow((1-triggerDeadZone), triggerLinearity) - Math.pow((left_trigger-triggerDeadZone), triggerLinearity)/Math.pow((1-triggerDeadZone), triggerLinearity); //look mommy an afront to coders everywhere (it also works first try :333)
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
            //Calculates speed by scaling hypotenuse through desired map
            double angle = Math.atan2(left_stick_y, left_stick_x);
            // Calculates Hypotenuse of triangle
            double scaleFactor = Math.sqrt(Math.pow(left_stick_x, 2) + Math.pow(left_stick_y, 2));
            //
            double ajustedScale = Math.pow((scaleFactor-joystickDeadZone), joystickLinearity)/Math.pow((1-joystickDeadZone), joystickLinearity);
            double forwardSpeed = ajustedScale * Math.sin(angle);
            double strafeSpeed = ajustedScale * Math.cos(angle);
            forward -= forwardSpeed;
            right += strafeSpeed;
        }

        //Slow strafe while holding x
        if (x) {
            maxMotorPower *= 0.5;
        }
        robot.lazyImu.get();
        double robotHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return HolonomicDrive.fieldOrientedDrive(right, forward, turn, maxMotorPower, robotHeading);
    }

}