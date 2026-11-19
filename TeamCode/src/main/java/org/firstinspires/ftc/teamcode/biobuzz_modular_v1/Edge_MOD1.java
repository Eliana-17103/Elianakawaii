package org.firstinspires.ftc.teamcode.biobuzz_modular_v1;

/** Rising-edge detector: rising(now) is true only on the loop where a button goes from released to pressed. */
public class Edge_MOD1 {
    private boolean prev = false;

    public boolean rising(boolean now) {
        boolean r = now && !prev;
        prev = now;
        return r;
    }
}
