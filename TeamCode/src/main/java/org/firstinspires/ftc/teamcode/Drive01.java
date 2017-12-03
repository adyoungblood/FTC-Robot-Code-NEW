package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Drive01", group = "Iterative Opmode")
public class Drive01 extends OpMode {

    public DcMotorController motor_controller_drive;
    public DcMotor motor_drive_right;
    public DcMotor motor_drive_left;

    private double power_motor_drive_right;
    private double power_motor_drive_left;

    private boolean button_y;
    private boolean button_LB;
    private boolean button_RB;
    private double button_RT;
    private double button_LT;

    @Override
    public void init() {
        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");

        motor_drive_right = hardwareMap.dcMotor.get("Motor_Drive_Right");
        motor_drive_left = hardwareMap.dcMotor.get("Motor_Drive_Left");

        motor_drive_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_drive_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gamepad1.setJoystickDeadzone((float) 0.1);

        motor_drive_right.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_left.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_drive_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_drive_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power_motor_drive_right = 0;
        power_motor_drive_left = 0;

        /*
        //noinspection deprecation
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.startupsoundxp, 1);
        mySound.play(beepID,1,1,1,0,1);
        */
    }

    @Override
    public void loop() {

        drive_with_joysticks();
    }

    public void drive_with_joysticks() {

        motor_drive_left.setPower(gamepad1.left_stick_y);
        motor_drive_right.setPower(gamepad1.right_stick_y);

        if (Math.pow(power_motor_drive_left, 3) > 0) {
            power_motor_drive_left = Math.max(Math.pow(power_motor_drive_left, 3), 0.7);
        } else if (Math.pow(power_motor_drive_left, 3) < 0) {
            power_motor_drive_left = Math.min(Math.pow(power_motor_drive_left, 3), -0.7);
        } else {
            power_motor_drive_left = 0;
        }
        if (Math.pow(power_motor_drive_right, 3) > 0) {
            power_motor_drive_right = Math.max(Math.pow(power_motor_drive_right, 3), 0.7);
        } else if (Math.pow(power_motor_drive_right, 3) < 0) {
            power_motor_drive_right = Math.min(Math.pow(power_motor_drive_right, 3), -0.7);
        } else {
            power_motor_drive_right = 0;
        }

        motor_drive_left.setPower(power_motor_drive_left);
        motor_drive_right.setPower(power_motor_drive_right);

    }

}