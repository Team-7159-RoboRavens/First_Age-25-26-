package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDriveGood;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FlywheelPDIFF;

@TeleOp(name="PIDFF TeloOp")
public class PIDFFTeleOp extends OpMode {
    //Global Variables
    ServoGoodBot robot;

    //Button Maps
    ServoAbstractButtonMapGood driveButtonMap;
    FlywheelPDIFF armButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new ServoGoodBot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new LiamPolarDriveGood();
        armButtonMap = new FlywheelPDIFF();
        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
        robot.runLimelight(24);
        telemetry.update();
    }
}
