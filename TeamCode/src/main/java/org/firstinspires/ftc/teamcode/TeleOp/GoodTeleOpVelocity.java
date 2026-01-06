package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.GoodVelocityDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDriveGood;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;

@TeleOp(name="GoodVelocityRed")
public class GoodTeleOpVelocity extends GoodTeleOpShared {
    ServoGoodBot robot;

    //Button Maps
    ServoAbstractButtonMapGood driveButtonMap;
    ServoAbstractButtonMapGood armButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new ServoGoodBot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new GoodVelocityDrive();
        armButtonMap = new FirstAgeGoodArm();
        telemetry.addLine("Ready.");
    }
    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
        robot.runLimelight(24);
        telemetry.update();
    }
}
