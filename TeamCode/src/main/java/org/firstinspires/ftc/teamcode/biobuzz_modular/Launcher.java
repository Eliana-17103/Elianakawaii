package org.firstinspires.ftc.teamcode.biobuzz_modular;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flywheel. Velocities are in encoder ticks/second.
 * Hub-side velocity PIDF (yours: 40, 0, 0, 12.5) with F scaled for battery voltage.
 * Call update() every loop.
 */
public class Launcher {
    // ---- tunables ----
    public static double P = 40, I = 0, D = 0, F_BASE = 12.5;
    public static double NOMINAL_VOLTAGE = 12.5;   // battery voltage F_BASE was tuned at
    public static double TOLERANCE_PCT = 0.03;     // +/-3% of target = "at speed"

    // driver-adjustable setting (TeleOp D-pad)
    public static double DEFAULT_SETTING = 1320, MIN_SETTING = 0, MAX_SETTING = 2500, INCREMENT = 10;

    private final DcMotorEx motor;
    private final VoltageSensor battery;
    private final ElapsedTime voltTimer = new ElapsedTime();
    private double target = 0, appliedVolts, setting = DEFAULT_SETTING;

    public Launcher(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "launcher");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        battery = hw.voltageSensor.iterator().next();
        applyPidf(battery.getVoltage());
    }

    private void applyPidf(double volts) {
        double scale = Range.clip(NOMINAL_VOLTAGE / Math.max(volts, 9.0), 0.85, 1.35);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F_BASE * scale));
        appliedVolts = volts;
    }

    public void update() {
        if (voltTimer.seconds() < 1.0) return;
        voltTimer.reset();
        double v = battery.getVoltage();
        if (Math.abs(v - appliedVolts) > 0.25) applyPidf(v);
    }

    // ---- control ----
    public void setVelocity(double ticksPerSec) { target = ticksPerSec; motor.setVelocity(ticksPerSec); }
    public void stop()                          { setVelocity(0); }
    public void runAtSetting()                  { setVelocity(setting); }

    // ---- driver setting ----
    public void bumpSetting(int direction) {
        setting = Range.clip(setting + direction * INCREMENT, MIN_SETTING, MAX_SETTING);
    }
    public double getSetting() { return setting; }

    // ---- status ----
    public double getVelocity() { return motor.getVelocity(); }
    public double getTarget()   { return target; }
    public boolean atSpeed() {
        return target > 0 && Math.abs(motor.getVelocity() - target) <= TOLERANCE_PCT * target;
    }
    public boolean isSpinning() { return motor.getVelocity() > 100; }

    public void addTelemetry(Telemetry t) {
        t.addLine("===== LAUNCHER =====");
        t.addData("Target / actual", "%.0f / %.0f", target, motor.getVelocity());
        t.addData("D-pad setting", "%.0f", setting);
        t.addData("At speed", atSpeed());
    }
}
