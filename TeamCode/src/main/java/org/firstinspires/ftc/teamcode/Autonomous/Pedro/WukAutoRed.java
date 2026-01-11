package org.firstinspires.ftc.teamcode.Autonomous.Pedro;

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

import org.firstinspires.ftc.teamcode.ButtonMaps.Arm.FirstAgeGoodArm;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "WukAutoRed")
public class WukAutoRed extends OpMode {
    public DcMotorEx ShootMotor;
    public DcMotorEx intakeMotor1;
    public DcMotorEx intakeMotor2;

    private Follower follower;
    private Timer stateTimer;

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

    Pose startPose   = new Pose(87.8, 8, Math.toRadians(90));
    Pose shootPose   = new Pose(83, 12, Math.toRadians(67.3));

    Pose pickPPGstart = new Pose(102, 35.3438, Math.toRadians(0));
    Pose pickPPGend   = new Pose(130, 35.3438, Math.toRadians(0));

    Pose pickPGPstart = new Pose(102, 60, Math.toRadians(0));
    Pose pickPGPend   = new Pose(130, 60, Math.toRadians(0));

    Pose gateClear   = new Pose(131.5, 62, Math.toRadians(-90));
    Pose loadingZone = new Pose(132, 11, Math.toRadians(0));
    Pose parkPose    = new Pose(92, 27, Math.toRadians(0));

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
                break;
            case SHOOT_1:
                // shooting code
                break;
            case SHOOT_TO_PICKUP_PPG:
                follower.followPath(shootToPickupPPG, true);
                break;
            case PICKUP_PPG_TO_PPGEND:
                //intake code
                follower.followPath(pickupPPGToPPGend, true);
                break;
            case PICKUP_PPGEND_TO_SHOOT:
                follower.followPath(pickupPPGendToShoot, true);
                break;
            case SHOOT_2:
                // shooting code
                break;
            case SHOOT_TO_PICKUP_PGP:
                follower.followPath(shootToPickupPGP, true);
                break;
            case PICKUP_PGP_TO_PGPEND:
                // intake code
                follower.followPath(pickupPGPToPGPend, true);
                break;
            case PGPEND_TO_GATE:
                follower.followPath(pgpendToGate, true);
                break;
            case GATE_CLEAR:
                break;
            case GATE_TO_SHOOT:
                follower.followPath(gateToShoot, true);
                break;
            case SHOOT_3:
                // shooting code
                break;
            case SHOOT_TO_LOAD:
                follower.followPath(shootToLoad, true);
                break;
            case LOAD_TO_SHOOT:
                follower.followPath(loadToShoot, true);
                break;
            case SHOOT_4:
                // shooting code
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
            if (stateTimer.getElapsedTimeSeconds() > 3.0)
                setState(AutoState.SHOOT_TO_PICKUP_PPG);
            break;
        case SHOOT_TO_PICKUP_PPG:
            if (!follower.isBusy())
                setState(AutoState.PICKUP_PPG_TO_PPGEND);
            break;
        case PICKUP_PPG_TO_PPGEND:
            if (!follower.isBusy())
                setState(AutoState.PICKUP_PPGEND_TO_SHOOT);
            break;
        case PICKUP_PPGEND_TO_SHOOT:
            if (!follower.isBusy())
                setState(AutoState.SHOOT_2);
            break;
        case SHOOT_2:
            while (stateTimer.getElapsedTimeSeconds() < 3.0) {
                double shootVel = ShootMotor.getVelocity();
                intakeMotor1.setPower(.8);
                intakeMotor2.setPower(.8);
                ShootMotor.setPower((FirstAgeGoodArm.velocityShot(196) - shootVel) / 137);
            }
            setState(AutoState.SHOOT_TO_PICKUP_PGP);
            break;
        case SHOOT_TO_PICKUP_PGP:
            if (!follower.isBusy())
                setState(AutoState.PICKUP_PGP_TO_PGPEND);
            break;
        case PICKUP_PGP_TO_PGPEND:
            if (!follower.isBusy())
                setState(AutoState.PGPEND_TO_GATE);
            break;
        case PGPEND_TO_GATE:
            if (!follower.isBusy())
                setState(AutoState.GATE_CLEAR);
            break;
        case GATE_CLEAR:
            while (stateTimer.getElapsedTimeSeconds() < 3.0) {
                double shootVel = ShootMotor.getVelocity();
                intakeMotor1.setPower(.8);
                intakeMotor2.setPower(.8);
                ShootMotor.setPower((FirstAgeGoodArm.velocityShot(196) - shootVel) / 137);
            }
            setState(AutoState.GATE_TO_SHOOT);
            break;
        case GATE_TO_SHOOT:
            if (!follower.isBusy())
                setState(AutoState.SHOOT_3);
            break;
        case SHOOT_3:
            while (stateTimer.getElapsedTimeSeconds() < 3.0) {
                double shootVel = ShootMotor.getVelocity();
                intakeMotor1.setPower(.8);
                intakeMotor2.setPower(.8);
                ShootMotor.setPower((FirstAgeGoodArm.velocityShot(196) - shootVel) / 137);
            }
            setState(AutoState.SHOOT_TO_LOAD);
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
            while (stateTimer.getElapsedTimeSeconds() < 3.0) {
                double shootVel = ShootMotor.getVelocity();
                intakeMotor1.setPower(.8);
                intakeMotor2.setPower(.8);
                ShootMotor.setPower((FirstAgeGoodArm.velocityShot(196) - shootVel) / 137);
            }
            setState(AutoState.PARK);
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
        ShootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Reset the motor encoder so that it reads zero ticks
        ShootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        ShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ShootMotor2 = hardwareMap.get(DcMotorEx.class, "ShootMotor2");
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intakeMotor1");
        intakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intakeMotor2");
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        follower = Constants.createFollower(hardwareMap);
        stateTimer = new Timer();
        buildPaths();
        follower.setPose(startPose);
        setState(AutoState.START_TO_SHOOT);
    }

    @Override
    public void loop() {
        follower.update();
        updateStateMachine();
    }
}
