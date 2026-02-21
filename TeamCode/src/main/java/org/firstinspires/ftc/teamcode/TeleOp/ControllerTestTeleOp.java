/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This tests each button on the controller by outputting their values. Useful for stick drift
 */
@TeleOp(name = "Controller testing")
public class ControllerTestTeleOp extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        telemetry.addLine("-------GAMEPAD 1--------");
        telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y: ", gamepad1.right_stick_y);
        telemetry.addData("Gamepad 1 A: ", gamepad1.a);
        telemetry.addData("Gamepad 1 B: ", gamepad1.b);
        telemetry.addData("Gamepad 1 X: ", gamepad1.x);
        telemetry.addData("Gamepad 1 Y: ", gamepad1.y);
        telemetry.addData("DPad Up: ", gamepad1.dpad_up);
        telemetry.addData("DPad Down: ", gamepad1.dpad_down);
        telemetry.addData("DPad Left: ", gamepad1.dpad_left);
        telemetry.addData("DPad Right: ", gamepad1.dpad_right);
        telemetry.addData("Left trigger: ", gamepad1.left_trigger);
        telemetry.addData("Right trigger: ", gamepad1.right_trigger);
        telemetry.addLine();
        telemetry.addLine("-------GAMEPAD 2--------");
        telemetry.addData("Left Stick X: ", gamepad2.left_stick_x);
        telemetry.addData("Left Stick Y: ", gamepad2.left_stick_y);
        telemetry.addData("Right Stick X: ", gamepad2.right_stick_x);
        telemetry.addData("Right Stick Y: ", gamepad2.right_stick_y);
        telemetry.addData("Gamepad 1 A: ", gamepad2.a);
        telemetry.addData("Gamepad 1 B: ", gamepad2.b);
        telemetry.addData("Gamepad 1 X: ", gamepad2.x);
        telemetry.addData("Gamepad 1 Y: ", gamepad2.y);
        telemetry.addData("DPad Up: ", gamepad2.dpad_up);
        telemetry.addData("DPad Down: ", gamepad2.dpad_down);
        telemetry.addData("DPad Left: ", gamepad2.dpad_left);
        telemetry.addData("DPad Right: ", gamepad2.dpad_right);
        telemetry.addData("Left trigger: ", gamepad2.left_trigger);
        telemetry.addData("Right trigger: ", gamepad2.right_trigger);
    }
}
