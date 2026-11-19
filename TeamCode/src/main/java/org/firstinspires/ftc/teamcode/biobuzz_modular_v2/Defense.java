package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Defense arm servo. Angles are degrees on a 300-degree servo.
 *
 *   Direct methods (TeleOp):  toggle(), setUp(boolean)
 *   Ivy commands (Auto):      raise(), lower()
 */
public class Defense {

    public static double UP_DEG = -30, DOWN_DEG = 90, RANGE_DEG = 300;

    private final Servo servo;
    private boolean up = false;

    public Defense(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "defense");
    }

    // ============ Direct methods (TeleOp) ============
    public void toggle() { setUp(!up); }

    public void setUp(boolean goUp) {
        up = goUp;
        double position = (up ? UP_DEG : DOWN_DEG) / RANGE_DEG;
        if (position < 0) position = 0;   // servo positions must be between 0 and 1
        if (position > 1) position = 1;
        servo.setPosition(position);
    }

    // ============ Ivy commands (Auto) ============
    public Command raise() { return instant(() -> setUp(true)); }
    public Command lower() { return instant(() -> setUp(false)); }

    public boolean isUp() { return up; }
    public String status() { return up ? "UP" : "DOWN"; }
}
