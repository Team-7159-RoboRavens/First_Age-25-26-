    package org.firstinspires.ftc.teamcode.Autonomous.Pedro;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

        @Autonomous (name="RedTriangleGPP")
        public class RedTriangleGPP extends OpMode {
            private Follower follower;
            private Timer pathTimer, opModeTimer;

            public enum PathState {
                //Start pos-end
                //Drive > move state
                //Shoot > attempt to score


                DRIVE_STARTPOS_PICK_POS,

                DRIVE_PICK_POS,
                SHOOT_PRELOAD
            }
            org.firstinspires.ftc.teamcode.Autonomous.Pedro.RedTrianglePGP.PathState pathState;

            private final Pose startPose = new Pose( 87.8982658, 8.15722543, Math.toRadians(90));

            private final Pose pickPose = new Pose( 100, 35, Math.toRadians(0));

            private final Pose pickPose2 = new Pose(128 , 35, Math.toRadians(0));

            private final Pose ballControlPose = new Pose(59.248658318425754, 63.37030411449016, Math.toRadians(0));

            private final Pose shootPose = new Pose(100 , 100, Math.toRadians(45));

            private PathChain driveStartPosPickPos, drivePickPosPickPos2, drivePickPos2ShootPos;

            public void buildPaths(){
                driveStartPosPickPos = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, pickPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), pickPose.getHeading())
                        .build();
                drivePickPosPickPos2 = follower.pathBuilder()
                        .addPath(new BezierLine(pickPose, pickPose2))
                        .setLinearHeadingInterpolation(pickPose.getHeading(), pickPose2.getHeading())
                        .build();
                drivePickPos2ShootPos = follower.pathBuilder()
                        .addPath(new BezierCurve(pickPose2, shootPose, ballControlPose))
                        .setLinearHeadingInterpolation(pickPose2.getHeading(), shootPose.getHeading())
                        .build();


            }

            public void statePathUpdate() {
                switch(pathState){
                    case DRIVE_STARTPOS_PICK_POS:
                        follower.followPath(driveStartPosPickPos, true);
                        if (!follower.isBusy()) {
                            setPathState(org.firstinspires.ftc.teamcode.Autonomous.Pedro.RedTrianglePGP.PathState.DRIVE_PICK_POS); //rest timer
                        }
                        break;
                    case DRIVE_PICK_POS:
                        follower.followPath(drivePickPosPickPos2, true);
                        if (!follower.isBusy()) {
                            setPathState(org.firstinspires.ftc.teamcode.Autonomous.Pedro.RedTrianglePGP.PathState.SHOOT_PRELOAD); //rest timer
                        }
                        break;
                    case SHOOT_PRELOAD:
                        // check if follower is done
                        follower.followPath(drivePickPos2ShootPos, true);
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 11) {
                            telemetry.addLine("done Path");

                            //add shooting code
                        }
                        break;
                    default:
                        telemetry.addLine("No State commanded");
                        break;
                }
            }

            public void setPathState(org.firstinspires.ftc.teamcode.Autonomous.Pedro.RedTrianglePGP.PathState newState) {
                pathState = newState;
                pathTimer.resetTimer();
            }

            @Override
            public void init()  {
                pathState = org.firstinspires.ftc.teamcode.Autonomous.Pedro.RedTrianglePGP.PathState.DRIVE_STARTPOS_PICK_POS;
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