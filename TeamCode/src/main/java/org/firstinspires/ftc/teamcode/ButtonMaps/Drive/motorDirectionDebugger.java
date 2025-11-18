package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

public class motorDirectionDebugger extends AbstractButtonMap {
    MotorPowers mp = new MotorPowers(0);
    @Override
    public void loop(FirstAgeTempbot robot, OpMode opMode) {

        if (opMode.gamepad1.a) {
            mp.leftFront = 1;
        }
        if (opMode.gamepad1.b) {
            mp.rightFront = 1;
        }
        if (opMode.gamepad1.x) {
            mp.leftBack = 1;
        }
        if (opMode.gamepad1.y) {
            mp.rightBack = 1;
        }

        opMode.telemetry.update();

        robot.setMotorPowers(mp);
}
}
