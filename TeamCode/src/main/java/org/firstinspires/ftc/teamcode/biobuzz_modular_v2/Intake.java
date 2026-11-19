package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake motor + left/right intake servos (right servo is reversed).
 *
 *   Direct methods (TeleOp):  setPower(p), halt(), setServosEnabled(on)
 *   Ivy commands (Auto):      collect(), eject(), off(), collectFor(ms)
 */
public class Intake {

    public static double COLLECT_POWER = 1.0;

    private final DcMotor motor;
    private final CRServo leftServo, rightServo;
    private boolean servosEnabled = true;   // set false when the flower mechanism is down
    private double power = 0;

    public Intake(HardwareMap hardwareMap) {
        motor      = hardwareMap.get(DcMotor.class, "intake");
        leftServo  = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        rightServo.setDirection(DcMotor.Direction.REVERSE);
    }

    // ============ Direct methods (TeleOp) ============
    public void setServosEnabled(boolean enabled) { servosEnabled = enabled; }

    /** Positive = pull balls in, negative = push balls out. */
    public void setPower(double p) {
        power = p;
        motor.setPower(p);
        if (servosEnabled) {
            leftServo.setPower(-p);
            rightServo.setPower(-p);
        }
    }

    public void halt() {
        power = 0;
        motor.setPower(0);
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    // ============ Ivy commands (Auto) ============
    public Command collect() { return instant(() -> setPower(COLLECT_POWER)); }
    public Command eject()   { return instant(() -> setPower(-COLLECT_POWER)); }
    public Command off()     { return instant(this::halt); }

    /** Intake for a set time, then stop. */
    public Command collectFor(int ms) {
        return sequential(collect(), waitMs(ms), off());
    }

    public String status() { return String.format("%.2f", power); }
}
