package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.DPadControl;
import org.firstinspires.ftc.teamcode.ButtonMaps.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;

@Config
public class HarshitaBM extends ServoAbstractButtonMap {
    private MotorPowers mp; // = new MotorPowers(0);
    double multiplier = 1; 
    // multiplier mainly for dpad speed control
    // also a test value for maxMotorPower parameter in HolonomicDrive.robotOrientedDrive() method
    double deadZone = 0.1;
    // dead zone for trigger and joystick

    @Override
    public void loop(ServoTempBot robot, OpMode opMode) {
        mp = new MotorPowers(0); 

        if (opMode.gamepad1.right_bumper) multiplier += 0.2; // increase speed by 0.2 when right bumper is pressed
        if (opMode.gamepad1.left_bumper) multiplier -= 0.2; // decrease speed by 0.2 when left bumper is pressed

        DPadControl.dpadStrafe(opMode, multiplier); // strafe when dpad is pressed

        if (opMode.gamepad1.left_trigger>deadZone)mp = HolonomicDrive.robotOrientedDrive(0,0,1,multiplier, opMode); // turn in place left when left trigger is pressed
        if (opMode.gamepad1.right_trigger>deadZone)mp = HolonomicDrive.robotOrientedDrive(0,0,-1,multiplier, opMode); // turn in place right when right trigger is pressed

        // NOTE: work on this later
        // Make it drive like a car when right joystick is used
//        if (Math.abs(opMode.gamepad1.right_stick_x) > deadZone || Math.abs(opMode.gamepad1.right_stick_y) > deadZone) {
//            // pythagorean theorem to calculate magnitude of joystick vector
//            double speed = Math.sqrt(opMode.gamepad1.right_stick_y * opMode.gamepad1.right_stick_y + opMode.gamepad1.right_stick_x * opMode.gamepad1.right_stick_x);
//
//            if (opMode.gamepad1.right_stick_x > deadZone || opMode.gamepad1.right_stick_y > deadZone) {
//                mp = HolonomicDrive.robotOrientedDrive(,,,multiplier * speed);
//            }
//        }

        if (Math.abs(opMode.gamepad1.left_stick_x) > deadZone || Math.abs(opMode.gamepad1.left_stick_y) > deadZone) mp = HolonomicDrive.JoystickHoloDrive(opMode.gamepad1, opMode); // holonomic drive with left joystick

        if (opMode.gamepad1.b) mp = new MotorPowers(0); // hard stop when b is pressed


        mp = new MotorPowers(mp.leftFront, mp.rightFront, mp.leftBack, mp.rightBack);

        opMode.telemetry.update();
        robot.setMotorPowers(mp);
    }
}