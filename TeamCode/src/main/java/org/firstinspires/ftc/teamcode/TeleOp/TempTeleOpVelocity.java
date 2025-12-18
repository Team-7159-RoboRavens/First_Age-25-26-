package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.GoodVelocityDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.TempVelocityDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMap;
import org.firstinspires.ftc.teamcode.ButtonMaps.ServoAbstractButtonMapGood;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoTempBot;
import org.firstinspires.ftc.teamcode.ShootingFunctions;

@TeleOp(name="TempVelocityRed")
public class TempTeleOpVelocity extends GoodTeleOpShared {
    ServoTempBot robot;

    //Button Maps
    ServoAbstractButtonMap driveButtonMap;
    ServoAbstractButtonMap armButtonMap;

    @Override
    public void init() {
        telemetry.addLine("Initializing, please wait...");
        telemetry.update();
        robot = new ServoTempBot(hardwareMap, new Pose2d(0,0,0), this);
        driveButtonMap = new TempVelocityDrive();
        armButtonMap = new FirstAgeArm();
        telemetry.addLine("Ready.");
        ShootingFunctions.currentPower = .7;
    }
    @Override
    public void loop() {
        driveButtonMap.loop(robot, this);
        armButtonMap.loop(robot, this);
        robot.runLimelight(24);
        telemetry.addLine("Shot power " + ShootingFunctions.currentPower);
        telemetry.update();
    }
}
