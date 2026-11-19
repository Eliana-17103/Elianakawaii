package org.firstinspires.ftc.teamcode.biobuzz_modular_v1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Transfer motor: feeds balls into the launcher. Supports hold-to-run and timed runs. */
public class Transfer_MOD1 {
    public static double POWER = -1.0;
    public static int RUN_MS = 3000;   // timed run length (~3 s after the launcher is at speed)

    private final DcMotor motor;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean running = false, timed = false;
    private int timedMs = RUN_MS;

    public Transfer_MOD1(HardwareMap hw) {
        motor = hw.get(DcMotor.class, "transfer");
    }

    public void run()  { motor.setPower(POWER); running = true; timed = false; }
    public void stop() { motor.setPower(0);     running = false; timed = false; }

    /** Run for RUN_MS then stop on its own. Call update() every loop for this to end. */
    public void startTimed() {
        run();
        timed = true;
        timedMs = RUN_MS;
        timer.reset();
    }

    public void update() {
        if (timed && timer.milliseconds() >= timedMs) stop();
    }

    public boolean isRunning() { return running; }

    public void addTelemetry(Telemetry t) {
        t.addData("Transfer", running ? (timed ? "RUNNING (timed)" : "RUNNING") : "off");
    }
}
