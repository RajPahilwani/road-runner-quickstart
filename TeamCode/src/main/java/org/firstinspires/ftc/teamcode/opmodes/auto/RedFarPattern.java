package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;

@Autonomous(name = "Red Far Pattern", group = "Auto")
public class RedFarPattern extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(60, 10, Math.toRadians(0));
    private static final Vector2d SCAN_POSE = new Vector2d(53, 13);

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
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(SCAN_POSE)
                        .turn(Math.toRadians(-377.5))
                        .build(),
                continuousPatternScan()
        ));

        doShotCycle(pattern, 3);

        safeStop();
    }
}
