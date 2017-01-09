package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SoundTest", group = "Iterative Opmode")
class SoundTest extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public SoundPool mySound;
    public int beepID;

    int timesPressed = 0;

    @Override
    public void init() {
        telemetry.clear();

        //noinspection deprecation
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.startupsoundxp, 1);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
            mySound.play(beepID,1,1,1,0,1);
            timesPressed = timesPressed + 1;
        }
        telemetry.addData("Times Pressed", timesPressed);
    }
}