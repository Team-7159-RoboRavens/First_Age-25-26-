package org.firstinspires.ftc.teamcode.ButtonMaps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.WheelTestAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.WheelTestBot;

public class TestButtonMap extends AbstractButtonMapTest {
    @Override
    public void loop(OpMode opMode) {
        opMode.telemetry.addLine("-------GAMEPAD 1--------");
        opMode.telemetry.addData("Left Stick X: ", opMode.gamepad1.left_stick_x);
        opMode.telemetry.addData("Left Stick Y: ", opMode.gamepad1.left_stick_y);
        opMode.telemetry.addData("Right Stick X: ", opMode.gamepad1.right_stick_x);
        opMode.telemetry.addData("Right Stick Y: ", opMode.gamepad1.right_stick_y);
        opMode.telemetry.addData("Gamepad 1 A: ", opMode.gamepad1.a);
        opMode.telemetry.addData("Gamepad 1 B: ", opMode.gamepad1.b);
        opMode.telemetry.addData("Gamepad 1 X: ", opMode.gamepad1.x);
        opMode.telemetry.addData("Gamepad 1 Y: ", opMode.gamepad1.y);
        opMode.telemetry.addData("DPad Up: ", opMode.gamepad1.dpad_up);
        opMode.telemetry.addData("DPad Down: ", opMode.gamepad1.dpad_down);
        opMode.telemetry.addData("DPad Left: ", opMode.gamepad1.dpad_left);
        opMode.telemetry.addData("DPad Right: ", opMode.gamepad1.dpad_right);
        opMode.telemetry.addData("Left trigger: ", opMode.gamepad1.left_trigger);
        opMode.telemetry.addData("Right trigger: ", opMode.gamepad1.right_trigger);
        opMode.telemetry.addLine("-------GAMEPAD 2--------");
        opMode.telemetry.addLine();
        opMode.telemetry.addData("Left Stick X: ", opMode.gamepad2.left_stick_x);
        opMode.telemetry.addData("Left Stick Y: ", opMode.gamepad2.left_stick_y);
        opMode.telemetry.addData("Right Stick X: ", opMode.gamepad2.right_stick_x);
        opMode.telemetry.addData("Right Stick Y: ", opMode.gamepad2.right_stick_y);
        opMode.telemetry.addData("Gamepad 1 A: ", opMode.gamepad2.a);
        opMode.telemetry.addData("Gamepad 1 B: ", opMode.gamepad2.b);
        opMode.telemetry.addData("Gamepad 1 X: ", opMode.gamepad2.x);
        opMode.telemetry.addData("Gamepad 1 Y: ", opMode.gamepad2.y);
        opMode.telemetry.addData("DPad Up: ", opMode.gamepad2.dpad_up);
        opMode.telemetry.addData("DPad Down: ", opMode.gamepad2.dpad_down);
        opMode.telemetry.addData("DPad Left: ", opMode.gamepad2.dpad_left);
        opMode.telemetry.addData("DPad Right: ", opMode.gamepad2.dpad_right);
        opMode.telemetry.addData("Left trigger: ", opMode.gamepad2.left_trigger);
        opMode.telemetry.addData("Right trigger: ", opMode.gamepad2.right_trigger);
        opMode.telemetry.update();
    }
}
