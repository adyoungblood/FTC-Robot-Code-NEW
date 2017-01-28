package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.math.*;

//import java.text.SimpleDateFormat;
//import java.util.Date;
/* (V 2.1 of drivetrain program)
 * Features:
 * Future proofed code for other, advanced functions (e.g. capping, pushing buttons, etc)
 * Basic program for Victanus's new drive train.
 * Basics:
 *  Forward - gamepad 2a, 1a = 1 (pressed) Motors left, right = 1
 * Turn right  - gamepad 2a, 1b = 1 (pressed) Motor right = -1, motor left = 1
 * Turn left  -  gamepad 2b, 1a = 1 (pressed) Motor right = 1, motor left = -1
 * Backward - gampad 2b, 1b = 1 (pressed) Motor right = -1, motor left = -1
 *  else - Motors left, right = 0
 *  Special Functions:
 *  Gamepad.x - button push - pushing x will push the button; default is retracted
 *  left stick y - capping - pushing upwards and downwards on the y-axis f controller will change
 *  the height of the capping mechanism
 *  right stick y - intake mechanism - pushing upwards
 *  Gamepad.b - shoot ball -
 *  <code that is commented out and indented will be added later>
 */
@TeleOp(name = "Drive02", group = "Iterative Opmode")
class Drive02 extends OpMode {

    GlobalFunctions g;

    @Override
    public void init() {
        telemetry.clear();
        // Initialize drive motors
        g.motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        g.motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");
        //capperMotor = hardwareMap.dcMotor.get("capperMotor")
        //
//        buttonServo = hardwareMap.servo.get("buttonServo");
        // If drive motors are given full power, robot would spin because of the motors being in
        // opposite directions. So reverse one
        g.motor_drive_left.setDirection(DcMotorSimple.Direction.REVERSE);
        g.motor_drive_right.setDirection(DcMotorSimple.Direction.FORWARD);
        //Declare positions of buttonServo //

        gamepad1.setJoystickDeadzone((float)0.2);


        //noinspection deprecation
        g.mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        g.beepID = g.mySound.load(hardwareMap.appContext, R.raw.startupsoundxp, 1);
        g.mySound.play(g.beepID,1,1,1,0,1);

        g.speed = false;

        g.startPrev = false;
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Initialized");

        g.startButton = gamepad1.start;
        g.left_trigger = -gamepad1.left_stick_y;
        g.right_trigger = -gamepad1.right_stick_y;

        if (g.startButton && !g.startPrev) {
            g.speed = true;
        } else {
            g.speed = false;
        }

        if (g.speed) {
            g.left_trigger = g.left_trigger / 2;
            g.right_trigger = g.right_trigger / 2;
            telemetry.addData("Speed", "Slow");
        } else {
            telemetry.addData("Speed", "Fast");
        }

        g.mySound.play(g.beepID,1,1,1,0,1);



        if (Math.pow(g.left_trigger, 3) > 0) {
            g.left_trigger = Math.max(Math.pow(g.left_trigger, 3), 0.7);
        } else if (Math.pow(g.left_trigger, 3) < 0) {
            g.left_trigger = Math.min(Math.pow(g.left_trigger, 3), -0.7);
        } else {
            g.left_trigger = 0;
        }

        if (Math.pow(g.right_trigger, 3) > 0) {
            g.right_trigger = Math.max(Math.pow(g.right_trigger, 3), 0.7);
        } else if (Math.pow(g.right_trigger, 3) < 0) {
            g.right_trigger = Math.min(Math.pow(g.right_trigger, 3), -0.7);
        } else {
            g.right_trigger = 0;
        }

        //Push button //
        if (gamepad1.a)
        {
  //          buttonServo.setPosition(ARM_EXTENDED_POSITION);

        }
        else {
            //arm resets to default //
    //        buttonServo.setPosition(ARM_RETRACTED_POSITION);
        }

        if (gamepad1.x) {
            g.left_trigger = 0;
            g.right_trigger = 0;
            driveFor(0.5, 1.5);
        }

        g.motor_drive_left.setPower(g.left_trigger);
        g.motor_drive_right.setPower(g.right_trigger);

        g.startPrev = g.startButton;
        telemetry.addData("Start Button", g.startButton);
        telemetry.addData("Start Previous", g.startPrev);
    }

    public void driveFor(double power, double seconds) {
        g.motor_drive_right.setPower(power);
        g.motor_drive_left.setPower(power);

        seconds = seconds * 1000;

        g.instant = g.runtime.milliseconds();
        while (g.instant > g.runtime.milliseconds() - seconds) {
            telemetry.addData("Time Left", (seconds - (g.runtime.milliseconds() - g.instant)));
        }

        g.motor_drive_left.setPower(0);
        g.motor_drive_right.setPower(0);
    }

}