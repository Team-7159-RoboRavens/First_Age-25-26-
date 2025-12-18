package org.firstinspires.ftc.teamcode.Autonomous.Pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name="BlueGoalPPG")
public class BlueGoalPGP extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //Start pos-end
        //Drive > move state
        //Shoot > attempt to score


        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        DRIVE_STARTPOS_SHOOT_POS2,

        DRIVE_STARTPOS_SHOOT_POS3,

        DRIVE_SHOOTPOS_ENDPOS;
    }
    PathState pathState;

    private final Pose startPose = new Pose( 20.789, 125, Math.toRadians(-40));

    private final Pose shootPose = new Pose( 72, 84, Math.toRadians(143));

    private final Pose shootPose2 = new Pose( 72, 84, Math.toRadians(180));

    private final Pose shootPose3 = new Pose( 15, 84, Math.toRadians(180));

    private final Pose shootPose4 = new Pose( 72, 23, Math.toRadians(115));


    private PathChain driveStartPosShootPos, driveShootPosEndPos,driveShootMovePos,driveShootMovePos2;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootMovePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, shootPose2))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose2.getHeading())
                .build();
        driveShootMovePos2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, shootPose3))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), shootPose3.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, shootPose4))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), shootPose4.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //rest timer
                break;
            case SHOOT_PRELOAD:
                // check if follower is done
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 11) {
                    telemetry.addLine("done Path");
                    setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS2);
                    //add shooting code
                }
                break;
            default:
                telemetry.addLine("No State commanded");
                break;
            case DRIVE_STARTPOS_SHOOT_POS2:
                if (!follower.isBusy()) {
                    telemetry.addLine("done Path3");
                }
                follower.followPath(driveShootMovePos, true);
                setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS3); //rest timer
                break;
            case DRIVE_STARTPOS_SHOOT_POS3:
                if (!follower.isBusy()) {
                    telemetry.addLine("done Path4");
                }
                follower.followPath(driveShootMovePos2, true);
                setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS); //rest timer
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                // check if follower is done
                if (!follower.isBusy()) {
                    telemetry.addLine("done Path2");
                }
                follower.followPath(driveShootPosEndPos, true);
                //add shooting code
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init()  {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);


        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop(){
        follower.update();
        statePathUpdate();
    }
}