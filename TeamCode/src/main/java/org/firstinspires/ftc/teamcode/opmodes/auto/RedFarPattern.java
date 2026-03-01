package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlanStuff.AutoBase;

@Autonomous(name = "Red Far Pattern", group = "Auto")
public class RedFarPattern extends AutoBase {

    private static final Pose2d START_POSE = new Pose2d(60, 10, Math.toRadians(0));
    private static final Vector2d SCAN_POSE = new Vector2d(53, 13);

    private static final long SHOT_SPINUP_MS = 1000;

    @Override
    public void runOpMode() {
        startPose = START_POSE;   // must set before initialize
        initialize();             // sets robot, drive, limelight

        robot.Tongue.setDown();
        robot.outake.intakeOff();
        robot.intake.intakeOff();

        // ===== Wait for start and show pattern continuously =====
        while (!isStarted() && !isStopRequested()) {
            pattern = detectPattern(pattern); // continuously update pattern
            telemetry.addLine("Ready for auto");
            telemetry.addData("Detected pattern", pattern);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) {
            safeStop();
            return;
        }

        // ===== Move to scan position while spinning full +17.5 turn and scanning pattern =====
        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(SCAN_POSE)
                        .turn(Math.toRadians(-270))
                        .turn(Math.toRadians(-90))
                        .turn(Math.toRadians(-17.5))
                        .build()
                //continuousPatternScan() // parallel Limelight scanning
        ));
        telemetry.addData("Got to before spinner", "pls");
        // ===== Spin up shooter =====
        Actions.runBlocking(spinUpShooter());
        telemetry.addData("Got to spinner", "pls");
        Actions.runBlocking(waitSeconds(SHOT_SPINUP_MS / 1000.0));

        // ===== Shoot all 3 balls safely according to pattern =====
        for (int i = 0; i < 3; i++) {
            char desiredChar = Character.toUpperCase(pattern.charAt(Math.min(i, 2)));
            Actions.runBlocking(rotateSpindexerIfWrongColor(desiredChar));
            telemetry.addData("got past rotating spindexer", "good");
            Actions.runBlocking(waitSeconds(1.5));

            Actions.runBlocking(new ParallelAction(raiseTongue(), waitSeconds(2.5)));

            Actions.runBlocking(new ParallelAction(lowerTongue(), waitSeconds(1)));

            Actions.runBlocking(rotateSpindexer());
        }

        Actions.runBlocking(new ParallelAction(raiseTongue(), waitSeconds(2)));

        Actions.runBlocking(new ParallelAction(lowerTongue(), waitSeconds(2)));


        safeStop();
    }
}