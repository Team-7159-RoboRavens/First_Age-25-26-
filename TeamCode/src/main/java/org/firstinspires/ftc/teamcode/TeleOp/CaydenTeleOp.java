package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.LimelightArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.CaydenPolarDrive;
import org.firstinspires.ftc.teamcode.ComplexRobots.CoachEricBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;

@TeleOp(name = "Cayden TeleOp")
public class CaydenTeleOp extends OpMode {
    //Global Variables
    CoachEricBot robot;

    //Button Maps
    AbstractButtonMap driveButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new CoachEricBot(hardwareMap, new Pose2d(0, 0, 0), this);
        driveButtonMap = new CaydenPolarDrive();
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.setServosTo(1, 1, 1, robot.intakeServo);
        driveButtonMap.loop(robot, this);
        telemetry.update();
    }
}
