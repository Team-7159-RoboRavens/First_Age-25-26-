package org.firstinspires.ftc.teamcode.Autonomous.TimeBased;

import static org.firstinspires.ftc.teamcode.Autonomous.TimeBased.GoalTimed.Direction.COMMON;
import static org.firstinspires.ftc.teamcode.Autonomous.TimeBased.GoalTimed.Direction.LEFT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.limelightData;

@Autonomous(name = "GoalTimedRed")
public class GoalTimedRed extends GoalTimed {
    public GoalTimedRed() {
        dir = Direction.LEFT;
        limelightData.ImuOffset = 2.17 - Math.PI/2;
    }
}