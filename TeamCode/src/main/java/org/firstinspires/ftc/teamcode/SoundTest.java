package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.*;

import static java.lang.Math.random;

@TeleOp(name = "SoundTest", group = "Iterative Opmode")
class SoundTest extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public SoundPool mySound;
    public int xpstartupsound;
    public int shadow;
    public int yee;
    public int screammeme;
    public int one;
    public int zylon;

    int timesPressed = 0;
    boolean backPrev = false;

    @Override
    public void init() {
        telemetry.clear();

        //noinspection deprecation
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        xpstartupsound = mySound.load(hardwareMap.appContext, R.raw.startupsoundxp, 1);
        shadow = mySound.load(hardwareMap.appContext, R.raw.shadow, 1);
        yee = mySound.load(hardwareMap.appContext, R.raw.yee, 1);
        screammeme = mySound.load(hardwareMap.appContext, R.raw.screammeme, 1);
        one = mySound.load(hardwareMap.appContext, R.raw.one, 1);
        zylon = mySound.load(hardwareMap.appContext, R.raw.zylon, 1);
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Initialized");
        if (gamepad1.back) {
            mySound.play(xpstartupsound, 1, 1, 1, 0, 1);
        }
        if (gamepad1.a) {
            mySound.play(shadow, 1, 1, 1, 0, 1);
        }
        if (gamepad1.b) {
            mySound.play(yee, 1, 1, 1, 0, 1);
        }
        if (gamepad1.y) {
            mySound.play(screammeme, 1, 1, 1, 0, 1);
        }
        if (gamepad1.x) {
            mySound.play(one, 1, 1, 1, 0, 1);
        }
        if (gamepad1.dpad_up) {
            mySound.play(zylon, 1, 1, 1, 0, 1);
        }
    }
}