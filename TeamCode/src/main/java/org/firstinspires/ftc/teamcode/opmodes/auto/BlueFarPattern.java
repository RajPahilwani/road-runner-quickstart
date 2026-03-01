package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;

@Autonomous(name = "Blue Far Pattern", group = "Auto")
public class BlueFarPattern extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(60, -10, Math.toRadians(0));
    private static final Vector2d SCAN_POSE = new Vector2d(53, -13);

    private static final long SHOT_SPINUP_MS = 1000;

    @Override
    public void runOpMode() {

        startPose = START_POSE;
        initialize();

        robot.Tongue.setDown();
        robot.outake.intakeOff();
        robot.intake.intakeOff();

        // ===== Wait for start and continuously update pattern =====
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

        // ===== Move to scan position =====
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(SCAN_POSE)
                        .turn(Math.toRadians(90))
                        .build()
        );

        pattern = detectPattern(pattern);

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(SCAN_POSE)
                        .turn(Math.toRadians(-90))
                        .turn(Math.toRadians(-17.5))
                        .build()
        );

        telemetry.addData("Got to before spinner", "pls");

        // ===== Spin up shooter =====
        Actions.runBlocking(spinUpShooterFar());
        telemetry.addData("Got to spinner", "pls");
        Actions.runBlocking(waitSeconds(SHOT_SPINUP_MS / 1000.0));

        // ===== Shoot all 3 balls according to pattern =====
        for (int i = 0; i < 3; i++) {

            char desiredChar = Character.toUpperCase(pattern.charAt(Math.min(i, 2)));

            Actions.runBlocking(rotateSpindexerIfWrongColor(desiredChar));
            telemetry.addData("got past rotating spindexer", "good");

            Actions.runBlocking(waitSeconds(1.5));

            Actions.runBlocking(new ParallelAction(
                    raiseTongue(),
                    waitSeconds(2.5)
            ));

            Actions.runBlocking(new ParallelAction(
                    lowerTongue(),
                    waitSeconds(1)
            ));

            Actions.runBlocking(rotateSpindexer());
        }

        safeStop();
    }
}