package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DPadControl {
    public static MotorPowers dpadStrafe(OpMode opMode, double multiplier) {
        MotorPowers mp = new MotorPowers(0);
        if (opMode.gamepad1.dpad_down) {
            //When dpad down-left is pressed, moves diagonally down-left
            if (opMode.gamepad1.dpad_left) mp = HolonomicDrive.robotOrientedDrive(-1, -1, 0, multiplier, opMode);
            //When dpad down-right is pressed, move diagonally down-right
            else if (opMode.gamepad1.dpad_right) mp = HolonomicDrive.robotOrientedDrive(1, -1, 0, multiplier, opMode);
            //When dpad down is pressed, move straight down
            else mp = HolonomicDrive.robotOrientedDrive(0, -1, 0, multiplier, opMode);
        } else if (opMode.gamepad1.dpad_up) {
            //When dpad up-left is pressed, move diagonally up-left
            if (opMode.gamepad1.dpad_left) mp = HolonomicDrive.robotOrientedDrive(-1, 1, 0, multiplier, opMode);
            //When dpad up-right is pressed, move diagonally up-right
            else if (opMode.gamepad1.dpad_right) mp = HolonomicDrive.robotOrientedDrive(1, 1, 0, multiplier, opMode);
            //When dpad up is pressed, move straight up
            else mp = HolonomicDrive.robotOrientedDrive(0, 1, 0, multiplier, opMode);
        }
        //When dpad right is pressed, move straight right
        else if (opMode.gamepad1.dpad_right) mp = HolonomicDrive.robotOrientedDrive(1, 0, 0, multiplier, opMode);
        //When dpad left is pressed, move straight left
        else if (opMode.gamepad1.dpad_left) mp = HolonomicDrive.robotOrientedDrive(-1, 0, 0, multiplier, opMode);

        return mp;
    }


}
