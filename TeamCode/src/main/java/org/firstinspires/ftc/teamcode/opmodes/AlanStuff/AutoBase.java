package org.firstinspires.ftc.teamcode.opmodes.AlanStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Outake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.List;

@Config
public abstract class AutoBase extends LinearOpMode {

    protected JVBoysSoccerRobot robot;
    protected Limelight3A limelight;
    protected Pose2d startPose;
    protected MecanumDrive drive;


    protected String pattern = "PPG";

    /* ===== CONSTANTS ===== */
    protected static final long SHOT_SPINUP_MS = 2000;
    protected static final long FEED_SETTLE_MS = 50;
    protected static final long PICKUP_TIMEOUT_MS = 1400;
    protected static final double HOLD_INTAKE_POWER = -0.4;

    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, true);

        drive = new MecanumDrive(hardwareMap, startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        robot.Tongue.setDown();
        robot.outake.intakeOff();
        robot.intake.intakeOff();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", RobotSettings.SUPER_TIME.toString());
        telemetry.update();
    }

    /* ===================== ATOMIC ACTIONS ===================== */
    protected Action spinUpShooterClose() {
        return packet -> {
            robot.turret.setAim(true);
            robot.outake.setPresetVelocity(Outake.CloseShotVelo);
            robot.outake.intakeOn();
            return false; // ends immediately, spinning is handled by waitSeconds
        };
    }
    protected Action spinUpShooterFar() {
        return packet -> {
            robot.turret.setAim(true);
            robot.outake.setPresetVelocity(Outake.FarShotVelo);
            robot.outake.intakeOn();
            return false; // ends immediately, spinning is handled by waitSeconds
        };
    }

    protected Action waitSeconds(double seconds) {
        final long start = System.currentTimeMillis();
        return packet -> {
            robot.update(true, true);
            return (System.currentTimeMillis() - start) < seconds * 1000; // true = keep running
        };
    }

    protected Action raiseTongue() {
        return packet -> { robot.Tongue.setUp(); return false; };
    }

    protected Action lowerTongue() {
        return packet -> { robot.Tongue.setDown(); return false; };
    }

    protected Action rotateSpindexer() {
        return packet -> {
            robot.spindexer.rotateByFraction(1.0 / 3.0);
            return false;
        };
    }

    protected Action rotateSpindexerIfWrongColor(char desiredChar) {

        final Spindexer.BallColor desiredColor =
                desiredChar == 'G'
                        ? Spindexer.BallColor.GREEN
                        : Spindexer.BallColor.PURPLE;

        final long start = System.currentTimeMillis();
        final long timeoutMs = 3000; // hard stop so it never hangs

        return packet -> {

            robot.update(true, true);

            // If no ball is seen, do not hang
            if (!robot.spindexer.seesBall()) {
                return false;
            }

            // If correct color is visible, we are done
            if (robot.spindexer.getVisibleBallColor() == desiredColor) {
                return false;
            }

            // Rotate only when idle
            if (robot.spindexer.isIdle()) {
                robot.spindexer.rotateByFraction(1.0 / 3.0);
            }

            // Hard timeout protection
            if (System.currentTimeMillis() - start > timeoutMs) {
                return false;
            }

            return true; // keep running
        };
    }

    protected Action waitForSpindexerIdle() {
        return packet -> { robot.update(true,true); return !robot.spindexer.isIdle(); };
    }

    protected Action waitForTongueDown() {
        return packet -> { robot.update(true,true); return !robot.Tongue.isDown(); };
    }

    protected Action holdIntakeWhileBallDetected(long timeoutMs) {
        final long start = System.currentTimeMillis();
        return packet -> {
            robot.intake.setPower(HOLD_INTAKE_POWER);
            robot.update(true,true);
            return robot.spindexer.seesBall() && (System.currentTimeMillis() - start < timeoutMs);
        };
    }

    protected Action continuousPatternScan() {
        return packet -> {
            pattern = detectPattern(pattern);
            return true; // always keep running
        };
    }

    protected Action continuousIntakeAndIndex(long timeoutMs) {
        final long start = System.currentTimeMillis();
        return packet -> {
            robot.update(true,true);
            robot.intake.setPower(1);

            if (robot.spindexer.seesBall() && robot.spindexer.isIdle()) {
                robot.spindexer.rotateByFraction(1.0 / 3.0);
            }

            return (System.currentTimeMillis() - start) < timeoutMs; // true = keep running until timeout
        };
    }

    /* ===================== COMPOSITE ACTIONS ===================== */
    protected void doShotCycle(String pattern, int shots) {


        Actions.runBlocking(spinUpShooterClose());
        Actions.runBlocking(waitSeconds(SHOT_SPINUP_MS / 1000.0));

        for (int i = 0; i < shots; i++) {
            char desiredChar = Character.toUpperCase(pattern.charAt(Math.min(i, 2)));
            Actions.runBlocking(rotateSpindexerIfWrongColor(desiredChar));
            Actions.runBlocking(waitForSpindexerIdle());

            Actions.runBlocking(raiseTongue());
            Actions.runBlocking(waitSeconds(FEED_SETTLE_MS / 1000.0));
            Actions.runBlocking(lowerTongue());
            Actions.runBlocking(waitForTongueDown());

            Actions.runBlocking(rotateSpindexer());
            Actions.runBlocking(waitForSpindexerIdle());
        }

        robot.outake.intakeOff();
        robot.turret.setAim(false);
    }

    protected void collectBallAt(Vector2d pickupPose) {
        robot.intake.intakeOn();
        robot.intake.setPower(1);

        Actions.runBlocking(new ParallelAction(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(pickupPose)
                        .build(),
                continuousIntakeAndIndex(PICKUP_TIMEOUT_MS)
        ));
    }

    /* ===================== PATTERN DETECTION ===================== */
    protected String detectPattern(String fallback) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return fallback;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return fallback;

        String detected = fallback;
        for (LLResultTypes.FiducialResult tag : tags) {
            switch (tag.getFiducialId()) {
                case 21: detected = "GPP"; break;
                case 22: detected = "PGP"; break;
                case 23: detected = "PPG"; break;
            }
        }
        return detected;
    }

    /* ===================== SAFE STOP ===================== */
    protected void safeStop() {
        robot.intake.intakeOff();
        robot.outake.intakeOff();
        robot.Tongue.setDown();
        limelight.stop();
        robot.update(true,true);
    }
}