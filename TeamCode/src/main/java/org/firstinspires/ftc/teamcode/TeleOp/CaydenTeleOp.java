//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.ButtonMaps.AbstractButtonMap;
//import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeArm;
//import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.LimelightArm;
//import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.CaydenPolarDrive;
//import org.firstinspires.ftc.teamcode.ComplexRobots.CoachEricBot;
//import org.firstinspires.ftc.teamcode.ComplexRobots.FirstAgeTempbot;
//
//@TeleOp(name = "Cayden TeleOp")
//public class CaydenTeleOp extends OpMode {
//    //Global Variables
//    CoachEricBot robot;
//
//    //Button Maps
//    AbstractButtonMap driveButtonMap;
//
//    @Override
//    public void init() {
//        telemetry.addLine("Initializing, please wait...");
//        telemetry.update();
//        robot = new CoachEricBot(hardwareMap, new Pose2d(0, 0, 0), this);
//        driveButtonMap = new CaydenPolarDrive();
//        telemetry.addLine("Ready!");
//        telemetry.update();
//    }
//    @Override
//    public void loop(FirstAgeTempbot robot, OpMode opMode) {
//        MotorPowers mp = getMotorPowers(
//                robot,
//                robot.lazyImu.get(),
//                opMode.gamepad1.dpad_up,
//                opMode.gamepad1.dpad_down,
//                opMode.gamepad1.dpad_left,
//                opMode.gamepad1.dpad_right,
//                opMode.gamepad1.left_bumper,
//                opMode.gamepad1.right_bumper,
//                opMode.gamepad1.left_trigger,
//                opMode.gamepad1.right_trigger,
//                opMode.gamepad1.left_stick_y,
//                opMode.gamepad1.left_stick_x,
//                opMode.gamepad1.x);
//
//        if (limelightData.aiming){
//            if (limelightData.accurate) {
//                opMode.telemetry.addLine("Aiming");
//                mp.leftFront -= limelightData.directionToTag()[0] * aimingPower;
//                mp.leftBack -= limelightData.directionToTag()[0] * aimingPower;
//                mp.rightFront += limelightData.directionToTag()[0] * aimingPower;
//                mp.rightBack += limelightData.directionToTag()[0] * aimingPower;
//            }
//            if (Math.abs(limelightData.directionToTag()[0]) < aimingThreshold) {
//                limelightData.aiming = false;
//                opMode.telemetry.addLine("Aimed");
//            }
//        }
//
//        opMode.telemetry.update();
//        robot.setMotorPowers(mp);
//    }
//    @Override
//    public void loop() {
//        robot.setServosTo(1, 1, 1, robot.intakeServo);
//        driveButtonMap.loop(robot, this);
//        telemetry.update();
//    }
//}
