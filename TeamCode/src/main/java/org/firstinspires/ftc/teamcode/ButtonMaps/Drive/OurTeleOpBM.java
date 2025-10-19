package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

public class OurTeleOpBM extends AbstractButtonMap {
    MotorPowers mp = new MotorPowers(0);
    @Override
    public void loop(FirstAgeTempbot robot, OpMode opMode) {
        if (Math.abs(opMode.gamepad1.left_stick_y) > .2) {
            mp = HolonomicDrive.fieldOrientedDrive(opMode.gamepad1, .8, robot.lazyImu.get());
        }
        else if (Math.abs(opMode.gamepad1.left_stick_x) > .2) {
            mp = HolonomicDrive.fieldOrientedDrive(opMode.gamepad1, .8, robot.lazyImu.get());
        }

        if (opMode.gamepad1.options) {
            robot.lazyImu.get().resetYaw();
        }

        opMode.telemetry.update();

        robot.setMotorPowers(mp);
}
}
