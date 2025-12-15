package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMapLimelight;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.LimelightArm;
import org.firstinspires.ftc.teamcode.ComplexRobots.LimelightBot;

@TeleOp(name="LimelightTeleOp")
public class LimelightTestTeleOp extends OpMode {
    //Global Variables
    LimelightBot robot;

    //Button Maps
//    AbstractButtonMap driveButtonMap;
    AbstractButtonMapLimelight armButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new LimelightBot(hardwareMap, new Pose2d(0,0,0), this);
//        driveButtonMap = new TempBotDrive();
        armButtonMap = new LimelightArm();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    //This code runs through the other files on a clock to respond to inputs.
    @Override
    public void loop() {
//        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
        //This id is the april tag ID for the red tower.
        robot.runLimelight(24);
        telemetry.update();
    }
}
