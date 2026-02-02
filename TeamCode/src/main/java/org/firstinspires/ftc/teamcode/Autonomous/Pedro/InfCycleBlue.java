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

@Autonomous(name = "InfCycleBlue")
public class InfCycleBlue extends OpMode {

    private Follower follower;
    private Timer stateTimer;
    private Timer autoTimer;
    public ServoGoodBot robot;

    private static final double SHOOT_TIME = 5.5;
    private static final double AUTO_END_TIME = 27.0;

    enum AutoState {
        START_TO_SHOOT,
        AIM,
        SHOOT,
        SHOOT_TO_PICKLOAD,
        PICKLOAD,
        PICKLOAD_TO_SHOOT,
        PARK,
        DONE
    }

    private AutoState state;

    Pose startPose   = new Pose(56.5, 8, Math.toRadians(90));
    Pose shootPose   = new Pose(61, 12, Math.toRadians(112.28));
    Pose pickLoadPose = new Pose(14.436, 7.5079, Math.toRadians(180));
    Pose parkPose = new Pose(48.0849, 22.407, Math.toRadians(180));

    PathChain startToShoot;
    PathChain shootToPickLoad;
    PathChain pickLoadToShoot;
    PathChain shootToPark;
    PathChain aim;

    void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToPickLoad = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickLoadPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickLoadPose.getHeading())
                .build();

        pickLoadToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickLoadPose, shootPose))
                .setLinearHeadingInterpolation(pickLoadPose.getHeading(), shootPose.getHeading())
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
                break;

            case AIM:
                follower.followPath(aim, true);
                break;

            case SHOOT:
                PedroFunctions.shoot(robot);
                break;

            case SHOOT_TO_PICKLOAD:
                PedroFunctions.intake(robot);
                follower.followPath(shootToPickLoad, true);
                break;

            case PICKLOAD:
                PedroFunctions.intake(robot);
                break;

            case PICKLOAD_TO_SHOOT:
                PedroFunctions.reset(robot);
                follower.followPath(pickLoadToShoot, true);
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
                if (!follower.isBusy()) {
                    aim = turn(Math.toRadians(limelightData.aprilXDegrees), follower);
                    setState(AutoState.AIM);
                }
                break;

            case AIM:
                if (!follower.isBusy()) {
                    setState(AutoState.SHOOT);
                }
                break;

            case SHOOT:
                if (stateTimer.getElapsedTimeSeconds() >= SHOOT_TIME) {
                    setState(AutoState.SHOOT_TO_PICKLOAD);
                    PedroFunctions.reset(robot);
                }
                break;

            case SHOOT_TO_PICKLOAD:
                if (!follower.isBusy())
                    setState(AutoState.PICKLOAD);
                break;

            case PICKLOAD:
                if (!follower.isBusy())
                    setState(AutoState.PICKLOAD_TO_SHOOT);
                break;

            case PICKLOAD_TO_SHOOT:
                if (!follower.isBusy()) {
                    aim = turn(Math.toRadians(limelightData.aprilXDegrees), follower);
                    setState(AutoState.AIM);
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
        autoTimer = new Timer();

        buildPaths();
        follower.setPose(startPose);

        robot = new ServoGoodBot(
                hardwareMap,
                new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading()),
                this
        );

        robot.ShootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(FlywheelPDIFF.P, 0, 0, FlywheelPDIFF.F)
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
        telemetry.update();
    }
}
