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
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name = "Drive02", group = "Iterative Opmode")
class Drive02 extends OpMode {

    GlobalFunctions g;

    @Override
    public void init() {
        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        // Initialize drive motors
        g.motor_drive_left = hardwareMap.dcMotor.get("Motor_Drive_Left");
        g.motor_drive_right = hardwareMap.dcMotor.get("Motor_Drive_Right");
        g.shooter_motor_1 = hardwareMap.dcMotor.get("Shooter_Motor_1");
        g.shooter_motor_2 = hardwareMap.dcMotor.get("Shooter_Motor_2");
        //g.motor_hat = hardwareMap.dcMotor.get("Hat_Motor");

        g.motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        g.motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        g.motor_controller_other = hardwareMap.dcMotorController.get("Motor_Controller_Other");
        //capperMotor = hardwareMap.dcMotor.get("capperMotor")
        //
        //buttonServo = hardwareMap.servo.get("buttonServo");
        // If drive motors are given full power, robot would spin because of the motors being in
        // opposite directions. So reverse one
        g.motor_drive_left.setDirection(DcMotorSimple.Direction.REVERSE);
        g.motor_drive_right.setDirection(DcMotorSimple.Direction.FORWARD);
        g.shooter_motor_1.setDirection(DcMotorSimple.Direction.FORWARD);
        g.shooter_motor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        //Declare positions of buttonServo //

        gamepad1.setJoystickDeadzone((float)0.2);

        /*
        //noinspection deprecation
        g.mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        g.beepID = g.mySound.load(hardwareMap.appContext, R.raw.startupsoundxp, 1);
        g.mySound.play(g.beepID,1,1,1,0,1);
        */

        g.speed = false;
        g.startPrev = false;
    }

    @Override
    public void loop() {
        telemetry.clear();
        telemetry.addData("Status", "In Loop");

        g.startButton = gamepad1.start;
        g.left_trigger = -gamepad1.left_stick_y;
        g.right_trigger = -gamepad1.right_stick_y;
        g.button_b = gamepad1.b;
        g.button_LB = gamepad1.left_bumper;
        g.button_RB = gamepad1.right_bumper;

        if (g.startButton && !g.startPrev) {
            g.speed = true;
        } else {
            g.speed = false;
        }



        //g.mySound.play(g.beepID,1,1,1,0,1);



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


        /*
        //Push button //
        if (gamepad1.a)
        {
  //          buttonServo.setPosition(ARM_EXTENDED_POSITION);

        }
        else {
            //arm resets to default //
    //        buttonServo.setPosition(ARM_RETRACTED_POSITION);
        }
        */


        /*
        if (gamepad1.x) {
            g.left_trigger = 0;
            g.right_trigger = 0;
            g.driveFor(0.5, 1.5);
        }
        */

        if (g.speed) {
            g.left_trigger = g.left_trigger / 2;
            g.right_trigger = g.right_trigger / 2;
            telemetry.addData("Speed", "Slow");
        } else {
            telemetry.addData("Speed", "Fast");
        }

        if (g.button_b) {
            g.shooter_motor_1.setPower(0.5);
            g.shooter_motor_2.setPower(0.5);
        } else {
            g.shooter_motor_1.setPower(0);
            g.shooter_motor_2.setPower(0);
        }

        if (g.button_LB) {
            g.intake_motor.setPower(1);
        } else {
            g.intake_motor.setPower(0);
        }

        if (g.button_RB) {
            g.belt_motor.setPower(1);
        } else {
            g.belt_motor.setPower(0);
        }

        telemetry.addData("Speed:", g.speed);
        telemetry.addData("Start Button:", g.startButton);
        telemetry.addData("Start Button Previous", g.startPrev);
        //telemetry.addData("Left DPad:", gamepad1.dpad_left);
        telemetry.addData("Left Trigger", g.left_trigger);
        telemetry.addData("Right Trigger", g.right_trigger);

        g.startPrev = g.startButton;

        g.motor_drive_left.setPower(g.left_trigger);
        g.motor_drive_right.setPower(g.right_trigger);
    }

}