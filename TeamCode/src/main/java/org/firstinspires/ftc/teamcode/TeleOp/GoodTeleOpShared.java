package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDriveGood;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;

public abstract class GoodTeleOpShared extends OpMode{
    //Global Variables
    ServoGoodBot robot;

    //Button Maps
    ServoAbstractButtonMapGood driveButtonMap;
    ServoAbstractButtonMapGood armButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new ServoGoodBot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new LiamPolarDriveGood();
        armButtonMap = new FirstAgeGoodArm();
        telemetry.addLine("Ready.");
    }
}
