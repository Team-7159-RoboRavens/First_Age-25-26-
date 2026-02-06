package org.firstinspires.ftc.teamcode.Autonomous.Pedro;

import static org.firstinspires.ftc.teamcode.Autonomous.Pedro.PedroFunctions.turn;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FlywheelPDIFF;
import org.firstinspires.ftc.teamcode.ComplexRobots.ServoGoodBot;
import org.firstinspires.ftc.teamcode.limelightData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "InfCycleRed")
public class InfCycleRed extends OpMode {

    private Follower follower;
    private Timer stateTimer;
    private Timer autoTimer;

    private static final double SHOOT_TIME = 4.2;
    private static final double INTAKE_TIME = 1.5;
    private static final double INTAKE_BURST_TIME = 0.5;
    private static final double AUTO_END_TIME = 27.0;

    ServoGoodBot robot;

    enum AutoState {
        START_TO_SHOOT,
        SHOOT,
        SHOOT_TO_PICKLOAD,
        PICKLOAD_INTAKE1,
        PICKLOAD_RECOIL,
        PICKLOAD_RETURN,
        PICKLOAD_END_TO_SHOOT,
        AIM,
        PARK,
        DONE
    }

    private AutoState state;

    Pose startPose = new Pose(87.8, 8, Math.toRadians(90));
    Pose shootPose = new Pose(80, 14, Math.toRadians(67.5));
    Pose pickLoadPoseEnd = new Pose(128, 8.900, Math.toRadians(-13));
    Pose pickLoadPoseRec = new Pose(116, 8.9, -13);
    Pose parkPose = new Pose(95.9161, 22.407, Math.toRadians(0));

    PathChain startToShoot;
    PathChain shootToPickLoad;
    PathChain shootToPark;
    PathChain pickLoadEndToRec;
    PathChain pickLoadRecToPickLoadEnd;
    PathChain pickLoadEndToShoot;
    PathChain aim;

    void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToPickLoad = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickLoadPoseEnd))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickLoadPoseEnd.getHeading())
                .build();

        shootToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();

        pickLoadEndToRec = follower.pathBuilder()
                .addPath(new BezierLine(pickLoadPoseEnd, pickLoadPoseRec))
                .setLinearHeadingInterpolation(pickLoadPoseEnd.getHeading(), pickLoadPoseRec.getHeading())
                .build();

        pickLoadRecToPickLoadEnd = follower.pathBuilder()
                .addPath(new BezierLine(pickLoadPoseRec, pickLoadPoseEnd))
                .setLinearHeadingInterpolation(pickLoadPoseRec.getHeading(), pickLoadPoseEnd.getHeading())
                .build();

        pickLoadEndToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickLoadPoseEnd, shootPose))
                .setLinearHeadingInterpolation(pickLoadPoseEnd.getHeading(), shootPose.getHeading())
                .build();
    }

    void setState(AutoState newState) {
        state = newState;
        stateTimer.resetTimer();

        switch (newState) {
            case START_TO_SHOOT:
                autoTimer.resetTimer();
                follower.followPath(startToShoot, true);
                break;
            case SHOOT:
                break;
            case SHOOT_TO_PICKLOAD:
                follower.followPath(shootToPickLoad, true);
                break;
            case PICKLOAD_INTAKE1:
                PedroFunctions.intake(robot);
                break;
            case PICKLOAD_RECOIL:
                PedroFunctions.intake(robot);
                follower.followPath(pickLoadEndToRec, true);
                break;
            case PICKLOAD_RETURN:
                PedroFunctions.intake(robot);
                follower.followPath(pickLoadRecToPickLoadEnd, true);
                break;
            case PICKLOAD_END_TO_SHOOT:
                PedroFunctions.intake(robot);
                follower.followPath(pickLoadEndToShoot, true);
                break;
            case AIM:
                follower.followPath(aim, true);
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
        if (autoTimer.getElapsedTimeSeconds() >= AUTO_END_TIME) {
            if (state != AutoState.PARK && state != AutoState.DONE) {
                setState(AutoState.PARK);
                return;
            }
        }

        switch (state) {
            case START_TO_SHOOT:
                if (!follower.isBusy()) setState(AutoState.SHOOT);
                break;

            case SHOOT:
                PedroFunctions.shoot(robot);
                if (stateTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    PedroFunctions.reset(robot);
                    setState(AutoState.SHOOT_TO_PICKLOAD);
                }
                break;

            case SHOOT_TO_PICKLOAD:
                if (!follower.isBusy()) setState(AutoState.PICKLOAD_INTAKE1);
                break;

            case PICKLOAD_INTAKE1:
                PedroFunctions.intake(robot);
                if (stateTimer.getElapsedTimeSeconds() >= INTAKE_BURST_TIME) {
                    setState(AutoState.PICKLOAD_RECOIL);
                }
                break;

            case PICKLOAD_RECOIL:
                if (!follower.isBusy()) setState(AutoState.PICKLOAD_RETURN);
                break;

            case PICKLOAD_RETURN:
                if (!follower.isBusy() || stateTimer.getElapsedTimeSeconds() >= INTAKE_TIME) {
                    setState(AutoState.PICKLOAD_END_TO_SHOOT);
                }
                break;

            case PICKLOAD_END_TO_SHOOT:
                if (!follower.isBusy()) {
                    aim = turn(Math.toRadians(limelightData.aprilXDegrees), follower, 80, Math.toRadians(67.5));
                    setState(AutoState.AIM);
                }
                break;

            case AIM:
                if (!follower.isBusy()) setState(AutoState.SHOOT);
                break;

            case PARK:
                PedroFunctions.reset(robot);
                if (!follower.isBusy()) setState(AutoState.DONE);
                break;

            case DONE:
                break;
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        stateTimer = new Timer();
        autoTimer = new Timer();
        follower.setPose(startPose);
        buildPaths();

        robot = new ServoGoodBot(
                hardwareMap,
                new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading()),
                this
        );

        robot.ShootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(FlywheelPDIFF.P + 1.2, 0, 0, FlywheelPDIFF.F + 1.2)
        );
    }

    @Override
    public void start() {
        stateTimer.resetTimer();
        autoTimer.resetTimer();
        setState(AutoState.START_TO_SHOOT);
    }

    @Override
    public void loop() {
        follower.update();
        updateStateMachine();
        telemetry.addData("Shoot Velocity", robot.ShootMotor.getVelocity());
        telemetry.addData("LimelightDegrees", limelightData.aprilXDegrees);
        robot.runLimelight(24);
        telemetry.update();
    }
}
