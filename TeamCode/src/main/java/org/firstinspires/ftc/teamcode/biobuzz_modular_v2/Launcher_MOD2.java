package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * The flywheel launcher. Speeds are in encoder ticks per second.
 *
 * TWO KINDS OF METHODS:
 *   Direct methods (TeleOp):  setVelocity(), halt(), bumpSetting()
 *   Ivy commands (Auto):      spinUp(), spinUp(tps), spinDown(), waitUntilReady()
 *
 * CALL update() EVERY LOOP (Auto and TeleOp). It handles battery sag, see below.
 *
 * BATTERY SAG: as the battery drains, the same motor settings spin the flywheel slower. The
 * feedforward term F is scaled by NOMINAL_VOLTAGE / current voltage, so the shot speed
 * stays about the same from a full battery to a tired one.
 */
public class Launcher_MOD2 {

    // ---- tunables ----
    public static double AUTO_SPEED = 1280;          // speed used by spinUp() in auto
    public static double START_SETTING = 1320;       // TeleOp speed on startup (D-pad changes it)
    public static double MIN_SETTING = 0, MAX_SETTING = 2500, STEP = 10;
    public static double AT_SPEED_TOLERANCE = 0.03;  // within 3% of target counts as "at speed"

    // PIDF (yours). F is the one that gets voltage-compensated.
    public static double P = 40, I = 0, D = 0, F = 12.5;
    public static double NOMINAL_VOLTAGE = 12.5;     // battery voltage when F was tuned. Change if you tuned at a different charge.

    // waitUntilReady()
    public static int STABLE_MS = 40;                // must stay at speed this long
    public static int READY_TIMEOUT_MS = 2500;       // gives up after this so auto never hangs

    private final DcMotorEx motor;
    private final VoltageSensor battery;
    private double target = 0;
    private double setting = START_SETTING;
    private double appliedVolts = 0;
    private int loopCount = 0;

    // for waitUntilReady()
    private long readyStartedAt = 0, atSpeedSince = -1;

    public Launcher_MOD2(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        battery = hardwareMap.voltageSensor.iterator().next();
        applyPidf(battery.getVoltage());
    }

    /** Sets PIDF with F scaled for the current battery voltage. */
    private void applyPidf(double volts) {
        double scale = NOMINAL_VOLTAGE / Math.max(volts, 9.0);   // never divide by a tiny/bad reading
        if (scale < 0.85) scale = 0.85;                          // keep the correction reasonable
        if (scale > 1.35) scale = 1.35;
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F * scale));
        appliedVolts = volts;
    }

    /** Call every loop. Checks the battery about once a second and re-adjusts F if it changed. */
    public void update() {
        loopCount++;
        if (loopCount < 50) return;       // reading voltage every loop would be wasteful
        loopCount = 0;
        double v = battery.getVoltage();
        if (Math.abs(v - appliedVolts) > 0.25) applyPidf(v);
    }

    // ============ Direct methods (TeleOp) ============
    public void setVelocity(double tps) { target = tps; motor.setVelocity(tps); }
    public void halt()                  { setVelocity(0); }

    /** Runs the flywheel at the driver's D-pad setting. */
    public void runAtSetting()          { setVelocity(setting); }

    /** direction: +1 faster, -1 slower. */
    public void bumpSetting(int direction) {
        setting += direction * STEP;
        if (setting > MAX_SETTING) setting = MAX_SETTING;
        if (setting < MIN_SETTING) setting = MIN_SETTING;
    }

    // ============ Ivy commands (Auto) ============
    public Command spinUp()           { return instant(() -> setVelocity(AUTO_SPEED)); }
    public Command spinUp(double tps) { return instant(() -> setVelocity(tps)); }
    public Command spinDown()         { return instant(this::halt); }

    /**
     * Finishes when the flywheel has held its target speed for STABLE_MS (or after READY_TIMEOUT_MS,
     * so auto can't get stuck). Use it instead of a fixed waitMs after spinUp().
     */
    public Command waitUntilReady() {
        return Command.build()
                .setStart(() -> { readyStartedAt = System.currentTimeMillis(); atSpeedSince = -1; })
                .setExecute(() -> {
                    if (atSpeed()) {
                        if (atSpeedSince < 0) atSpeedSince = System.currentTimeMillis();
                    } else {
                        atSpeedSince = -1;
                    }
                })
                .setDone(() -> {
                    long now = System.currentTimeMillis();
                    boolean stable = atSpeedSince >= 0 && now - atSpeedSince >= STABLE_MS;
                    return stable || now - readyStartedAt >= READY_TIMEOUT_MS;
                });
    }

    // ============ Status ============
    public double getVelocity() { return motor.getVelocity(); }
    public double getTarget()   { return target; }
    public double getSetting()  { return setting; }
    public boolean isSpinning() { return motor.getVelocity() > 100; }

    public boolean atSpeed() {
        return target > 0 && Math.abs(motor.getVelocity() - target) <= AT_SPEED_TOLERANCE * target;
    }

    public String status() {
        return String.format("target %.0f / actual %.0f / setting %.0f / %.1fV%s",
                target, motor.getVelocity(), setting, appliedVolts, atSpeed() ? "  [AT SPEED]" : "");
    }
}
