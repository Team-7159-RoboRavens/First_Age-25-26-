package org.firstinspires.ftc.teamcode.ButtonMaps.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ButtonMaps.WheelTestAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.WheelTestBot;

public class motorDirectionDebugger extends WheelTestAbstractButtonMap {
    @Override
    public void loop(WheelTestBot robot, OpMode opMode) {
        MotorPowers mp = new MotorPowers(0);

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

        if (Math.abs(opMode.gamepad1.right_stick_x) > 0.1 || Math.abs(opMode.gamepad1.right_stick_y) > 0.1) {
            double speed = .6 * Math.sqrt(opMode.gamepad1.right_stick_y * opMode.gamepad1.right_stick_y + opMode.gamepad1.right_stick_x * opMode.gamepad1.right_stick_x);
            double speedX = speed - 2 * opMode.gamepad1.right_stick_x * opMode.gamepad1.right_stick_x;
            double speedY = speed - 2 * opMode.gamepad1.right_stick_y * opMode.gamepad1.right_stick_y;
            if (opMode.gamepad1.right_stick_x >= 0 && opMode.gamepad1.right_stick_y <= 0) {
                mp = new MotorPowers(speed,
                        speedX,
                        speedX,
                        speed);
            } else if (opMode.gamepad1.right_stick_x >= 0 && opMode.gamepad1.right_stick_y >= 0) {
                mp = new MotorPowers(speedY,
                        -speed,
                        -speed,
                        speedY);
            } else if (opMode.gamepad1.right_stick_x <= 0 && opMode.gamepad1.right_stick_y >= 0) {
                mp = new MotorPowers(-speed,
                        speedY,
                        speedY,
                        -speed);
            } else {
                mp = new MotorPowers(speedX,
                        speed,
                        speed,
                        speedX);
            }
        }
        opMode.telemetry.addData("LeftFront ", mp.leftFront);
        opMode.telemetry.addData("RightFront ", mp.rightFront);
        opMode.telemetry.addData("LeftBack ", mp.leftBack);
        opMode.telemetry.addData("RightBack ", mp.rightBack);
        opMode.telemetry.addData("LeftFront Vel", robot.leftFront.getVelocity());
        opMode.telemetry.addData("LeftBack Vel", robot.leftBack.getVelocity());
        opMode.telemetry.addData("RightFront Vel", robot.rightFront.getVelocity());
        opMode.telemetry.addData("RightBack Vel", robot.rightBack.getVelocity());

        opMode.telemetry.update();

        robot.setMotorPowers(mp);
    }
}
