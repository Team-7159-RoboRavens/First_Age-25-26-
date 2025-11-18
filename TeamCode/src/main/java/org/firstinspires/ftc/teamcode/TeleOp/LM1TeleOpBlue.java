package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;

@TeleOp(name="LM1TeleopBlue")
public class LM1TeleOpBlue extends LM1TeleOpShared {
    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonmap.loop(robot, this);
        robot.runLimelight(25);
        telemetry.update();
    }
}
