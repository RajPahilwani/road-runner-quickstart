package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outake extends Subsystem {

    /* ===== Hardware ===== */
    private final DcMotorEx leader;
    private final DcMotorEx follower;
    private final Telemetry telemetry;

    /* ===== Preset Velocities (RPM) ===== */
    public static int FarShotVelo = 3800;
    public static int MediumShotVelo = 1680;
    public static int CloseShotVelo = 2800;
    public static int AutoFarShotVelo = 1650;

    /* ===== State ===== */
    private Mode mode = Mode.OFF;
    private int currentVelo = FarShotVelo; // default preset

    private enum Mode {
        ON,
        OFF
    }

    public Outake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leader = hwMap.get(DcMotorEx.class, "fly");
        follower = hwMap.get(DcMotorEx.class, "fly1");

        leader.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        follower.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        follower.setDirection(DcMotorEx.Direction.REVERSE);
        leader.setDirection(DcMotorEx.Direction.REVERSE);

        leader.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        follower.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /* ===== Commands ===== */
    public void intakeOn() {
        mode = Mode.ON;
    }

    public void intakeOff() {
        stop();
    }

    public void setPresetVelocity(int presetRPM) {
        currentVelo = presetRPM;
    }

    /* ===== Update Loop ===== */
    @Override
    public void update() {
        double targetVelocity = rpmToTicksPerSec(currentVelo);

        switch (mode) {
            case ON:
                // Both motors should spin at the same power level to maintain the same speed
                leader.setVelocity(targetVelocity);
                follower.setVelocity(targetVelocity); // Follower matches leader's speed
                break;

            case OFF:
            default:
                stop();
                break;
        }
    }

    /* ===== Stop Method ===== */
    @Override
    public void stop() {
        leader.setPower(0);
        follower.setPower(0);
        mode = Mode.OFF;
    }

    /* ===== Helpers ===== */
    private double rpmToTicksPerSec(double rpm) {
        return rpm * 28 / 60.0; // TICKS_PER_REV = 28, convert RPM to TICKS per second
    }

    private double getRPM() {
        return leader.getVelocity() * 60.0 / 28;
    }

    /* ===== Telemetry ===== */
    @Override
    public void addTelemetry() {
        telemetry.addLine("Flywheel");
        telemetry.addData("Mode", mode);
        telemetry.addData("Target RPM", currentVelo);
        telemetry.addData("Actual RPM", getRPM());
        telemetry.addData("Leader Velocity", leader.getVelocity());
        telemetry.addData("Follower Velocity", follower.getVelocity());
    }
}
