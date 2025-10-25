package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@Config
public class HarshitaBM extends AbstractButtonMap {

    private MotorPowers mp; // = new MotorPowers(0);

    @Override
    public void loop(FirstAgeTempbot robot, OpMode opMode) {
        mp = new MotorPowers(0);
        double multiplier = 1; // mainly for the dpad

        //Increases speed (multiplier) when right bumper is pressed by 0.1
        if (opMode.gamepad1.right_bumper) {
            multiplier += 0.1;
        }
        //Decreases speed (multiplier) when left bumper is pressed by 0.1
        if (opMode.gamepad1.left_bumper) {
            multiplier -= 0.1;
        }

        // Hard stop when b is pressed
        if (opMode.gamepad1.b) {
            mp = new MotorPowers(0);
        }


        // When left trigger is pressed, turn left while staying in place
        if (opMode.gamepad1.left_trigger>0.1){
            mp = new MotorPowers(1 * opMode.gamepad1.left_trigger,
                    -1 * opMode.gamepad1.left_trigger,
                    1 * opMode.gamepad1.left_trigger,
                    -1 * opMode.gamepad1.left_trigger);
        }
        // When right trigger is pressed, turn right while staying in place
        if (opMode.gamepad1.right_trigger>0.1){
            mp = new MotorPowers(-1 * multiplier * opMode.gamepad1.right_trigger,
                    1 * opMode.gamepad1.right_trigger,
                    -1 * opMode.gamepad1.right_trigger,
                    1 * opMode.gamepad1.right_trigger);
        }


        if (opMode.gamepad1.dpad_down) {
            //When dpad down-left is pressed, moves diagonally down-left
            if (opMode.gamepad1.dpad_left) {
                mp = new MotorPowers(1 * multiplier,
                        0 * multiplier,
                        0 * multiplier,
                        1 * multiplier);
            }
            //When dpad down-right is pressed, move diagonally down-right
            else if (opMode.gamepad1.dpad_right){
                mp = new MotorPowers(0 * multiplier,
                        1 * multiplier,
                        1 * multiplier,
                        0 * multiplier);
            }
            //When dpad down is pressed, move straight down
            else {
                mp = new MotorPowers(1 * multiplier,
                        1 * multiplier,
                        1 * multiplier,
                        1 * multiplier);
            }
        } else if (opMode.gamepad1.dpad_up) {
            //When dpad up-left is pressed, move diagonally up-left
            if (opMode.gamepad1.dpad_left) {
                mp = new MotorPowers(0 * multiplier,
                        -1 * multiplier,
                        -1 * multiplier,
                        0 * multiplier);
            }
            //When dpad up-right is pressed, move diagonally up-right
           else if (opMode.gamepad1.dpad_right) {
                mp = new MotorPowers(-1 * multiplier,
                        0 * multiplier,
                        0 * multiplier,
                        -1 * multiplier);
            //When dpad up is pressed, move straight up
            } else {
                mp = new MotorPowers(-1 * multiplier,
                        -1 * multiplier,
                        -1 * multiplier,
                        -1 * multiplier);
            }
           //When dpad right is pressed, move strafe right
        } else if (opMode.gamepad1.dpad_right){
            mp = new MotorPowers(1 * multiplier,
                    -1 * multiplier,
                    -1 * multiplier,
                    1 * multiplier);
            //When dpad left is pressed, move straight left
        } else if (opMode.gamepad1.dpad_left){
            mp = new MotorPowers(-1 * multiplier,
                    1 * multiplier,
                    1 * multiplier,
                    -1 * multiplier);
        }
       /* if (Math.abs(opMode.gamepad1.left_stick_x) > 0.2 || Math.abs(opMode.gamepad1.left_stick_y) > 0.2){
            mp = HolonomicDrive.JoystickHoloDrive(opMode.gamepad1);
        } */


        // Allows Joystick to control where the robot goes
        /*if (Math.abs(opMode.gamepad1.right_stick_x) > 0.4 || Math.abs(opMode.gamepad1.right_stick_y) > 0.4) {
            // pythagorean theorem to calculate magnitude of joystick vector
            double speed = Math.sqrt(opMode.gamepad1.right_stick_y * opMode.gamepad1.right_stick_y + opMode.gamepad1.right_stick_x * opMode.gamepad1.right_stick_x);

            if (opMode.gamepad1.right_stick_x >= 0.1 && opMode.gamepad1.right_stick_y <= 0.1) {
                // first quadrant

            } else if (opMode.gamepad1.right_stick_x >= 0.1 && opMode.gamepad1.right_stick_y >= 0.1) {
                // fourth quadrant

            } else if (opMode.gamepad1.right_stick_x <= 0.1 && opMode.gamepad1.right_stick_y  >= 0.1) {
                // third quadrant

            } else {
                // second quadrant

            }
            //mp = HolonomicDrive.JoystickHoloDrive(opMode.gamepad1);
        } */
        // holonomic drive with left joystick
        if (Math.abs(opMode.gamepad1.left_stick_x) > 0.4 || Math.abs(opMode.gamepad1.left_stick_y) > 0.4) {
            mp = HolonomicDrive.JoystickHoloDrive(opMode.gamepad1, opMode);
        }

        mp = new MotorPowers(mp.leftFront, mp.rightFront, mp.leftBack, mp.rightBack);

        opMode.telemetry.update();
        robot.setMotorPowers(mp);
    }
}