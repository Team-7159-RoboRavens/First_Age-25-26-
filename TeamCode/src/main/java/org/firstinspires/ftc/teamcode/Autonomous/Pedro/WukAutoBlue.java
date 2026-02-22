package org.firstinspires.ftc.teamcode.Autonomous.Pedro;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FlywheelPDIFF;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDrive;
import org.firstinspires.ftc.teamcode.ButtonMaps.Drive.LiamPolarDriveGood;
import org.firstinspires.ftc.teamcode.ButtonMaps.MotorPowers;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.DualLogger;
import org.firstinspires.ftc.teamcode.limelightData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Autonomous.Pedro.PedroFunctions;

@Autonomous(name = "WukAutoBlue")
public class WukAutoBlue extends OpMode {

    private Follower follower;
    private final DualLogger dualLogger = new DualLogger(telemetry);
    private Timer stateTimer;
    public DcMotorEx ShootMotor;
    public DcMotorEx intakeMotor1;
    public DcMotorEx intakeMotor2;

    public ServoGoodBot robot;
    static double aimingThreshold = .06;

    enum AutoState {
        START_TO_SHOOT,
        SHOOT_1,
        SHOOT_TO_PICKUP_PPG,
        PICKUP_PPG_TO_PPGEND,
        PICKUP_PPGEND_TO_SHOOT,
        SHOOT_2,
        SHOOT_TO_PICKUP_PGP,
        PICKUP_PGP_TO_PGPEND,
        PGPEND_TO_GATE,
        GATE_CLEAR,
        GATE_TO_SHOOT,
        SHOOT_3,
        SHOOT_TO_LOAD,
        LOAD_TO_SHOOT,
        SHOOT_4,
        PARK,
        DONE
    }

    private AutoState state;

    Pose startPose = new Pose(56.5, 8, Math.toRadians(90));
    Pose shootPose = new Pose(61, 12, Math.toRadians(112.7));

    Pose pickPPGstart = new Pose(46.41798, 30.3438, Math.toRadians(180));
    Pose pickPPGend = new Pose(17, 30.3438, Math.toRadians(180));

    Pose pickPGPstart = new Pose(46.64277, 55, Math.toRadians(180));
    Pose pickPGPend = new Pose(17.5, 55, Math.toRadians(180));

    Pose gateClear = new Pose(13.5, 60, Math.toRadians(270));
    Pose loadingZone = new Pose(12, 11, Math.toRadians(180));
    Pose parkPose = new Pose(52, 27, Math.toRadians(180));

    PathChain startToShoot;
    PathChain shootToPickupPPG;
    PathChain pickupPPGToPPGend;
    PathChain pickupPPGendToShoot;
    PathChain shootToPickupPGP;
    PathChain pickupPGPToPGPend;
    PathChain pgpendToGate;
    PathChain gateToShoot;
    PathChain shootToLoad;
    PathChain loadToShoot;
    PathChain shootToPark;

    void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToPickupPPG = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickPPGstart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickPPGstart.getHeading())
                .build();

        pickupPPGToPPGend = follower.pathBuilder()
                .addPath(new BezierLine(pickPPGstart, pickPPGend))
                .setLinearHeadingInterpolation(pickPPGstart.getHeading(), pickPPGend.getHeading())
                .build();

        pickupPPGendToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickPPGend, shootPose))
                .setLinearHeadingInterpolation(pickPPGend.getHeading(), shootPose.getHeading())
                .build();

        shootToPickupPGP = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickPGPstart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickPGPstart.getHeading())
                .build();

        pickupPGPToPGPend = follower.pathBuilder()
                .addPath(new BezierLine(pickPGPstart, pickPGPend))
                .setLinearHeadingInterpolation(pickPGPstart.getHeading(), pickPGPend.getHeading())
                .build();

        pgpendToGate = follower.pathBuilder()
                .addPath(new BezierLine(pickPGPend, gateClear))
                .setLinearHeadingInterpolation(pickPGPend.getHeading(), gateClear.getHeading())
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gateClear, shootPose))
                .setLinearHeadingInterpolation(gateClear.getHeading(), shootPose.getHeading())
                .build();

        shootToLoad = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, loadingZone))
                .setLinearHeadingInterpolation(shootPose.getHeading(), loadingZone.getHeading())
                .build();

        loadToShoot = follower.pathBuilder()
                .addPath(new BezierLine(loadingZone, shootPose))
                .setLinearHeadingInterpolation(loadingZone.getHeading(), shootPose.getHeading())
                .build();

        shootToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    void setState(AutoState newState) {
        state = newState;
        stateTimer.resetTimer();

        switch (newState) {
            case START_TO_SHOOT:
                follower.followPath(startToShoot, true);
                PedroFunctions.aim(robot);
                break;
            case SHOOT_1:
                break;
            case SHOOT_TO_PICKUP_PPG:
                PedroFunctions.intake(robot);
                follower.followPath(shootToPickupPPG, true);
                break;
            case PICKUP_PPG_TO_PPGEND:
                //probably won't work because intake needs the robot to move simultaneously
                PedroFunctions.intake(robot);
                follower.followPath(pickupPPGToPPGend, true);
                break;
            case PICKUP_PPGEND_TO_SHOOT:
                follower.followPath(pickupPPGendToShoot, true);
                PedroFunctions.aim(robot);
                break;
            case SHOOT_2:
                break;
            case SHOOT_TO_PICKUP_PGP:
                PedroFunctions.intake(robot);
                follower.followPath(shootToPickupPGP, true);
                break;
            case PICKUP_PGP_TO_PGPEND:
                PedroFunctions.intake(robot);
                follower.followPath(pickupPGPToPGPend, true);
                break;
            case PGPEND_TO_GATE:
                follower.followPath(pgpendToGate, true);
                break;
            case GATE_CLEAR:
                break;
            case GATE_TO_SHOOT:
                follower.followPath(gateToShoot, true);
                PedroFunctions.aim(robot);
                break;
            case SHOOT_3:
                break;
            case SHOOT_TO_LOAD:
                PedroFunctions.intake(robot);
                follower.followPath(shootToLoad, true);
                break;
            case LOAD_TO_SHOOT:
                PedroFunctions.reset(robot);
                follower.followPath(loadToShoot, true);
                break;
            case SHOOT_4:
                break;
            case PARK:
                follower.followPath(shootToPark, true);
                break;
            case DONE:
                follower.breakFollowing();
                break;
        }
    }

    void updateStateMachine() {
        switch (state) {
            case START_TO_SHOOT:
                if (!follower.isBusy())
                    setState(AutoState.SHOOT_1);
                break;
            case SHOOT_1:
                if (!follower.isBusy()) {
                    PedroFunctions.shoot(robot);
//                PedroFunctions.aim(robot);
                    if (stateTimer.getElapsedTimeSeconds() >= 3.0) {
                        setState(AutoState.SHOOT_TO_PICKUP_PPG);
                        PedroFunctions.reset(robot);
                    }
                }
                break;
            case SHOOT_TO_PICKUP_PPG:
                if (!follower.isBusy())
                    setState(AutoState.PICKUP_PPG_TO_PPGEND);
                break;
            case PICKUP_PPG_TO_PPGEND:
                PedroFunctions.intake(robot);
                if (!follower.isBusy()) {
                    setState(AutoState.PICKUP_PPGEND_TO_SHOOT);
                    PedroFunctions.reset(robot);
                }
                break;
            case PICKUP_PPGEND_TO_SHOOT:
                if (!follower.isBusy())
                    setState(AutoState.SHOOT_2);
                break;
            case SHOOT_2:
                if (!follower.isBusy()) {
                    PedroFunctions.shoot(robot);
//                PedroFunctions.aim(robot);
                    if (stateTimer.getElapsedTimeSeconds() >= 3.0) {
                        setState(AutoState.SHOOT_TO_PICKUP_PGP);
                        PedroFunctions.reset(robot);
                    }
                }
                break;
            case SHOOT_TO_PICKUP_PGP:
                PedroFunctions.intake(robot);
                if (!follower.isBusy()) {
                    setState(AutoState.PICKUP_PGP_TO_PGPEND);
                    PedroFunctions.reset(robot);
                }
                break;
            case PICKUP_PGP_TO_PGPEND:
                if (!follower.isBusy() || stateTimer.getElapsedTimeSeconds() >= .7)
                    setState(AutoState.PGPEND_TO_GATE);
                break;
            case PGPEND_TO_GATE:
                if (!follower.isBusy() || stateTimer.getElapsedTimeSeconds() >= .7)
                    setState(AutoState.GATE_CLEAR);
                break;
            case GATE_CLEAR:
                if (!follower.isBusy() || stateTimer.getElapsedTimeSeconds() >= 1) {
                    setState(AutoState.GATE_TO_SHOOT);
                    PedroFunctions.reset(robot);
                }
                break;
            case GATE_TO_SHOOT:
                if (!follower.isBusy())
                    setState(AutoState.SHOOT_3);
                break;
            case SHOOT_3:
                if (!follower.isBusy()) {
                    PedroFunctions.shoot(robot);
//                PedroFunctions.aim(robot);
                    if (stateTimer.getElapsedTimeSeconds() >= 3.0) {
                        setState(AutoState.SHOOT_TO_LOAD);
                        PedroFunctions.reset(robot);
                    }
                }
                break;
            case SHOOT_TO_LOAD:
                if (!follower.isBusy())
                    setState(AutoState.LOAD_TO_SHOOT);
                break;
            case LOAD_TO_SHOOT:
                if (!follower.isBusy())
                    setState(AutoState.SHOOT_4);
                break;
            case SHOOT_4:
                if (!follower.isBusy()) {
                    PedroFunctions.shoot(robot);
//                PedroFunctions.aim(robot);
                    if (stateTimer.getElapsedTimeSeconds() >= 3.0) {
                        setState(AutoState.PARK);
                        PedroFunctions.reset(robot);
                    }
                }
                break;
            case PARK:
                if (!follower.isBusy())
                    setState(AutoState.DONE);
                break;
            case DONE:
                break;
        }
    }


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        stateTimer = new Timer();
        buildPaths();
        follower.setPose(startPose);
        setState(AutoState.START_TO_SHOOT);
        robot = new ServoGoodBot(hardwareMap, new Pose2d(0, 0, 0), this, new DualLogger(telemetry));
        robot.shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FlywheelPDIFF.P, 0, 0, FlywheelPDIFF.F));

    }

    @Override
    public void loop() {
        follower.update();
        updateStateMachine();
        dualLogger.addData("Shoot Velocity", robot.shootMotor.getVelocity());
        dualLogger.addData("LimelightDegrees", limelightData.aprilXDegrees);
        robot.runLimelight(20);
        telemetry.update();
    }
}
