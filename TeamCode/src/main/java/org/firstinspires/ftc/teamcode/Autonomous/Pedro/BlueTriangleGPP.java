package org.firstinspires.ftc.teamcode.Autonomous.Pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name="BlueGoalGETOUT!!!!!")
public class BlueTriangleGPP extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //Start pos-end
        //Drive > move state
        //Shoot > attempt to score

        DRIVE_STARTPOS_PREPPOS,
        DRIVE_PREPPOS_BALLPOS,
        DRIVE_BALLPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        SHOOT_PRELOAD2,
        DRIVE_SHOOTPOS_ENDPOS
    }
    PathState pathState;

    private final Pose startPose = new Pose( 60, 13, Math.toRadians(90));
    private final Pose prepPose = new Pose( 50, 35, Math.toRadians(90));

    private final Pose ballPose = new Pose( 20, 35, Math.toRadians(0));

    private final Pose shootPose = new Pose( 50, 8, Math.toRadians(-60));

    private PathChain driveStartPosPrepPos,drivePrepPosBallPos, driveBallPosShootPos;

    public void buildPaths(){
        driveStartPosPrepPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, prepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), prepPose.getHeading())
                .build();
        drivePrepPosBallPos = follower.pathBuilder()
                .addPath(new BezierLine(prepPose, ballPose))
                .setLinearHeadingInterpolation(prepPose.getHeading(), ballPose.getHeading())
                .build();
        driveBallPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPose, shootPose))
                .setLinearHeadingInterpolation(ballPose.getHeading(), shootPose.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_PREPPOS:
                follower.followPath(driveStartPosPrepPos, true);
                setPathState(PathState.DRIVE_PREPPOS_BALLPOS); //rest timer
                break;
            case DRIVE_PREPPOS_BALLPOS:
                follower.followPath(drivePrepPosBallPos, true);
                setPathState(PathState.DRIVE_BALLPOS_SHOOTPOS); //rest timer
                break;
            case DRIVE_BALLPOS_SHOOTPOS:
                follower.followPath(driveBallPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //rest timer
                break;
            case SHOOT_PRELOAD2:
                // check if follower is done
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 11) {
                    //add shooting code different
                }
                break;
            case SHOOT_PRELOAD:
                // check if follower is done
                if (pathTimer.getElapsedTimeSeconds() > 11) {
                    //add shooting code
                }
                setPathState(PathState.DRIVE_STARTPOS_PREPPOS);
                break;
            default:
                telemetry.addLine("No State commanded");
                break;
            case DRIVE_SHOOTPOS_ENDPOS:
                // check if follower is done
                if (!follower.isBusy()) {
                    telemetry.addLine("done Path");
                    //add shooting code
                }
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init()  {
        pathState = PathState.SHOOT_PRELOAD;
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