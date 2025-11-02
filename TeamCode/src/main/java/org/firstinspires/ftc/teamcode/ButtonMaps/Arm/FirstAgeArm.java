package org.firstinspires.ftc.teamcode.ButtonMaps.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.limelightData;
import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@Config
public class FirstAgeArm extends ServoAbstractButtonMap {
    //TODO: Change back to private final when done with dash
    private MotorPowers mp;// = new MotorPowers(0);
    private double servoPosition;
    private double timeSince;
    private double timeBuffer = 200;
    private double servoSpeed = .5;

    @Override
    public void loop(ServoTempBot robot, OpMode opMode) {

        //Automatically Aim if there is a tag
        if (opMode.gamepad2.x && opMode.getRuntime() - timeSince > timeBuffer) {
            if (limelightData.accurate) {
                timeSince = opMode.getRuntime();
                limelightData.aiming = true;
//                robot.setServosTo(-1, 1, limelightData.directionToTag()[0], robot.aimServo);
//                robot.setServosTo(-1, 1, limelightData.directionToTag()[1], robot.angleServo);
                opMode.telemetry.addLine("Aiming");
            }
            else{
                opMode.telemetry.addLine("No Tag");
                limelightData.aiming = false;
            }

        }

            robot.Servo1.setPower(opMode.gamepad2.left_stick_y);
            robot.Servo2.setPower(opMode.gamepad2.left_stick_y);
            opMode.telemetry.addData("Servos Going", opMode.gamepad2.left_stick_y);

        if (opMode.gamepad2.a) {
            robot.Servo1.setPower(0);
            robot.Servo2.setPower(0);
            opMode.telemetry.addLine("Servos stop");

        }

        //Aim manually with the left joystick
        if (Math.abs(opMode.gamepad2.left_stick_y) > 0.2 || Math.abs(opMode.gamepad2.left_stick_x) > 0.2) {
            //                robot.setServosTo(-1, 1, opMode.gamepad2.left_stick_x, robot.aimServo);
            //                robot.setServosTo(-1, 1, opMode.gamepad2.left_stick_y, robot.angleServo);
            opMode.telemetry.addLine("Aiming manually, x/y: " + opMode.gamepad2.left_stick_x + opMode.gamepad2.left_stick_y);
        }






        opMode.telemetry.update();
    }
}
