package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;

@Autonomous(name = "Red Auto Goal Pattern", group = "Auto")
public class RedAutoGoalPattern extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(-52, 49, Math.toRadians(127.5));
    private static final Vector2d SCAN_AND_SHOOT_POSE = new Vector2d(-12, 15);
    private static final Vector2d PICKUP_1 = new Vector2d(-12, 37);
    private static final Vector2d PICKUP_2 = new Vector2d(-12, 43);
    private static final Vector2d PICKUP_3 = new Vector2d(-12, 52.5);

    @Override
    public void runOpMode() {
        startPose = START_POSE;
        initialize();

        while (!isStarted() && !isStopRequested()) {
            pattern = detectPattern(pattern);
            telemetry.addLine("Ready for auto");
            telemetry.addData("Detected pattern", pattern);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            safeStop();
            return;
        }

        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(START_POSE)
                        .strafeTo(SCAN_AND_SHOOT_POSE)
                        .build(),
                continuousPatternScan()
        ));

        doShotCycle(pattern, 3);

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(-37.5))
                        .build()
        );

        collectBallAt(PICKUP_1);
        collectBallAt(PICKUP_2);
        collectBallAt(PICKUP_3);

        doShotCycle(pattern, 3);

        safeStop();
    }
}
