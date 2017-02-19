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

@TeleOp(name = "Drive03_Sylvana", group = "Iterative Opmode")
public class Drive03_Sylvana extends OpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor shooter_motor_2;
    public DcMotorController motor_controller_belt;
    public DcMotor belt_motor;
    public DcMotor intake_motor;
    public Servo intake_servo;
    public ServoController intake_servo_controller;
    public DcMotorController motor_controller_drive;
    public DcMotor motor_drive_right;
    public DcMotor motor_drive_left;

    public double power_motor_drive_right;
    public double power_motor_drive_left;

    public boolean button_y;
    public boolean button_LB;
    public boolean button_RB;
    public boolean button_LB2;
    public boolean button_RB2;
    public double button_RT;
    public double button_LT;
    public boolean shooting;

    double instant;

    @Override
    public void init() {
        motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        motor_controller_belt = hardwareMap.dcMotorController.get("Motor_Controller_Belt");
        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        intake_servo_controller = hardwareMap.servoController.get("Servo_Controller_Intake");

        shooter_motor_1 = hardwareMap.dcMotor.get("Motor_Shooter_1");
        shooter_motor_2 = hardwareMap.dcMotor.get("Motor_Shooter_2");

        belt_motor = hardwareMap.dcMotor.get("Motor_Belt");
        intake_motor = hardwareMap.dcMotor.get("Motor_Intake");

        intake_servo = hardwareMap.servo.get("Servo_Intake");

        motor_drive_right = hardwareMap.dcMotor.get("Motor_Drive_Right");
        motor_drive_left = hardwareMap.dcMotor.get("Motor_Drive_Left");

        shooter_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter_motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_drive_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_drive_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        belt_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepad1.setJoystickDeadzone((float) 0.1);
        shooter_motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter_motor_2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_drive_right.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_left.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_drive_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_drive_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        belt_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        power_motor_drive_right = 0;
        power_motor_drive_left = 0;
        intake_servo.setPosition(0);

        /*
        //noinspection deprecation
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.startupsoundxp, 1);
        mySound.play(beepID,1,1,1,0,1);
        */

        button_LB = false;
        button_LB2 = false;
        button_RB = false;
        button_RB2 = false;
        button_y = false;
        shooting = false;
    }

    @Override
    public void loop() {

//        drive_with_joystick();
        drive_with_buttons();
        shooter_control();
        intake_control();
        servo_control();
    }

    public void drive_with_joystick() {

        motor_drive_left.setPower(gamepad1.left_stick_y);
        motor_drive_right.setPower(gamepad1.right_stick_y);

/*
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

        if (speed) {
            power_motor_drive_left = power_motor_drive_left / 2;
            power_motor_drive_right = power_motor_drive_right / 2;
            telemetry.addData("Speed", "Slow");
        } else {
            telemetry.addData("Speed", "Fast");
        }

        motor_drive_left.setPower(power_motor_drive_left);
        motor_drive_right.setPower(power_motor_drive_right);

        startPrev = startButton;
*/
    }

    public void intake_control() {

        button_RB2 = gamepad2.right_bumper;

        if (button_RB) {
            intake_motor.setPower(1);
            belt_motor.setPower(0.3);
        } else if (!shooting) {
            intake_motor.setPower(0);
            belt_motor.setPower(0);
        }
    }

    public void shooter_control() {

        button_LB2 = gamepad2.left_bumper;

        if (button_LB) {
            shooter_motor_1.setPower(0.225);
            shooter_motor_2.setPower(0.225);
        } else if (!shooting) {
            shooter_motor_1.setPower(0);
            shooter_motor_2.setPower(0);
        }


        if (button_y) {
            shooting = true;
            shoot(0.47, 0.3, -0.03);
            shooting = false;
        }
    }

    public void servo_control() {

        button_y = gamepad2.y;

        if (button_y) {
            intake_servo.setPosition(0.05);
        } else if (!shooting) {
            intake_servo.setPosition(0);
        }
    }

    public void drive_with_buttons() {

        // Define mode in which power is controlled:

        button_RT = gamepad1.right_trigger;
        button_RB = gamepad1.right_bumper;

        if (button_RT > 0) {
            motor_drive_right.setPower(button_RT);
        } else if (button_RB) {
            motor_drive_right.setPower(-0.5);
        } else {
            motor_drive_right.setPower(0);
        }

        button_LT = gamepad1.left_trigger;
        button_LB = gamepad1.left_bumper;

        if (button_LT > 0) {
            motor_drive_left.setPower(button_LT);
        } else if (button_LB) {
            motor_drive_left.setPower(-0.5);
        } else {
            motor_drive_left.setPower(0);
        }
    }


    public void waitFor(double seconds) {
        motor_drive_left.setPower(0);
        motor_drive_right.setPower(0);
        try {
            TimeUnit.MILLISECONDS.sleep((long) seconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }



    public void shoot(double shoot_power, double belt_power, double acceleration) {
        intake_servo.setPosition(0);
        shooter_motor_1.setPower(shoot_power);
        shooter_motor_2.setPower(shoot_power);
        waitFor(2);
        belt_motor.setPower(belt_power);
        waitFor(0.5);
        intake_servo.setPosition(0.05);
        waitFor(1);
        shooter_motor_1.setPower(shoot_power + acceleration);
        shooter_motor_2.setPower(shoot_power + acceleration);
        waitFor(1);
        intake_servo.setPosition(0);
        shooter_motor_1.setPower(0);
        shooter_motor_2.setPower(0);
        belt_motor.setPower(0);
    }
}