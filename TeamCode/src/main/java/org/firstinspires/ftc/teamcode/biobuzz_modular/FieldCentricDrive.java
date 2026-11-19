package org.firstinspires.ftc.teamcode.biobuzz_modular;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Mecanum drive, field-centric. Pass in the driver-frame heading (see HeadingFrame). */
public class FieldCentricDrive {
    public static double DEADZONE = 0.05;
    public static double NORMAL_SPEED = 1.0, SLOW_SPEED = 0.35;
    public static double STRAFE_BOOST = 1.1;

    private final DcMotor leftFront, leftBack, rightFront, rightBack;
    private boolean slowMode = false;
    private double lastFl, lastBl, lastFr, lastBr;

    public FieldCentricDrive(HardwareMap hw) {
        leftFront  = hw.get(DcMotor.class, "left_front_drive");
        leftBack   = hw.get(DcMotor.class, "left_back_drive");
        rightFront = hw.get(DcMotor.class, "right_front_drive");
        rightBack  = hw.get(DcMotor.class, "right_back_drive");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void toggleSlowMode() { slowMode = !slowMode; }
    public boolean isSlowMode()  { return slowMode; }

    /** Largest stick deflection; used to detect "driver wants control back". */
    public static double stickMagnitude(Gamepad g) {
        return Math.max(Math.abs(g.left_stick_x),
               Math.max(Math.abs(g.left_stick_y), Math.abs(g.right_stick_x)));
    }

    /**
     * @param g            driver gamepad
     * @param driveHeading heading (rad) of the driver's frame, with any "back is front" offset already applied
     */
    public void drive(Gamepad g, double driveHeading) {
        double y  = -g.left_stick_y;
        double x  = g.left_stick_x * STRAFE_BOOST;
        double rx = -g.right_stick_x;
        if (Math.abs(y)  < DEADZONE) y = 0;
        if (Math.abs(x)  < DEADZONE) x = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        double rotX = x * Math.cos(-driveHeading) - y * Math.sin(-driveHeading);
        double rotY = x * Math.sin(-driveHeading) + y * Math.cos(-driveHeading);

        double fl = rotY + rotX + rx;
        double bl = rotY - rotX + rx;
        double fr = rotY - rotX - rx;
        double br = rotY + rotX - rx;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                                            Math.max(Math.abs(fr), Math.abs(br))));
        double s = (slowMode ? SLOW_SPEED : NORMAL_SPEED) / max;
        lastFl = fl * s; lastBl = bl * s; lastFr = fr * s; lastBr = br * s;

        leftFront.setPower(lastFl);  leftBack.setPower(lastBl);
        rightFront.setPower(lastFr); rightBack.setPower(lastBr);
    }

    public void stop() {
        leftFront.setPower(0); leftBack.setPower(0);
        rightFront.setPower(0); rightBack.setPower(0);
    }

    public void addTelemetry(Telemetry t) {
        t.addLine("===== DRIVE =====");
        t.addData("Slow mode", slowMode ? "ON" : "OFF");
        t.addData("FL / BL", "%.2f / %.2f", lastFl, lastBl);
        t.addData("FR / BR", "%.2f / %.2f", lastFr, lastBr);
    }
}
