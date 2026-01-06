package org.firstinspires.ftc.teamcode.Autonomous.TimeBased;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.limelightData;

@Autonomous(name = "GoalTimedBlue")
public class GoalTimedBlue extends GoalTimed {
    public GoalTimedBlue() {
        limelightData.ImuOffset = Math.PI/12 + 2.17 - 5 * Math.PI/4;
        dir = Direction.RIGHT;
    }
}