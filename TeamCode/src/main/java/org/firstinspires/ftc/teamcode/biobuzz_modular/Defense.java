package org.firstinspires.ftc.teamcode.biobuzz_modular;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Defense arm servo. Angles are in degrees on a 300-degree servo; positions are clamped to 0..1. */
public class Defense {
    public static double UP_DEG = -30, DOWN_DEG = 90, RANGE_DEG = 300;

    private final Servo servo;
    private boolean up = false;

    public Defense(HardwareMap hw) {
        servo = hw.get(Servo.class, "defense");
    }

    public void toggle() {
        up = !up;
        servo.setPosition(Range.clip((up ? UP_DEG : DOWN_DEG) / RANGE_DEG, 0.0, 1.0));
    }

    public boolean isUp() { return up; }

    public void addTelemetry(Telemetry t) {
        t.addData("Defense", up ? "UP" : "DOWN");
    }
}
