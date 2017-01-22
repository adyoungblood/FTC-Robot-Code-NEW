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
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor intake_motor_1;
    public DcMotor intake_motor_2;
    public Servo buttonServo;
    public DcMotorController motor_controller_drive;
    public DcMotor motor_drive_left;
    public DcMotor motor_drive_right;

    // Initilization of drive train variables:
    //public double power_forward;
    public double power_back;
    //    public double power_RT;
//    public double power_LT;
    public double power_level;

    // Initilization of drive train variables:
    //public double power_forward;
    public double power_shooter;

    // Initialization of joystick buttons:
    public double button_RT;
    public double button_LT;

    public boolean button_RB;
    public boolean button_LB;
/*    public boolean button_a;
    public boolean button_y;
    public boolean button_b;
    public boolean button_x;*/

    //    public double joystick1_right_x;
    public double joystick1_right_y;
    public double joystick1_left_x;
    public double joystick1_left_y;
    public double joystick1_right_x;
    public double joystick2_right_y;

    public double ARM_RETRACTED_POSITION = 0.2;
    public double ARM_EXTENDED_POSITION = 0.8;

    public double left_train_power;
    public double right_train_power;

    public boolean startButton;
    public boolean startPrev;
    public boolean speed;

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    public SoundPool mySound;
    public int beepID;

    double left_trigger;
    double right_trigger;

    double instant;

    @Override
    public void init() {
        telemetry.clear();
        // Initialize drive motors
        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");
        //capperMotor = hardwareMap.dcMotor.get("capperMotor")
        //
//        buttonServo = hardwareMap.servo.get("buttonServo");
        // If drive motors are given full power, robot would spin because of the motors being in
        // opposite directions. So reverse one
        motor_drive_left.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_right.setDirection(DcMotorSimple.Direction.FORWARD);
        //Declare positions of buttonServo //

        gamepad1.setJoystickDeadzone((float)0.2);


        //noinspection deprecation
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.startupsoundxp, 1);
        mySound.play(beepID,1,1,1,0,1);

        speed = false;

        startPrev = false;
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Initialized");

        startButton = gamepad1.start;
        left_trigger = -gamepad1.left_stick_y;
        right_trigger = -gamepad1.right_stick_y;

        if (startButton && !startPrev) {
            speed = true;
        } else {
            speed = false;
        }

        if (speed) {
            left_trigger = left_trigger / 2;
            right_trigger = right_trigger / 2;
            telemetry.addData("Speed", "Slow");
        } else {
            telemetry.addData("Speed", "Fast");
        }

        mySound.play(beepID,1,1,1,0,1);



        if (Math.pow(left_trigger, 3) > 0) {
            left_trigger = Math.max(Math.pow(left_trigger, 3), 0.7);
        } else if (Math.pow(left_trigger, 3) < 0) {
            left_trigger = Math.min(Math.pow(left_trigger, 3), -0.7);
        } else {
            left_trigger = 0;
        }

        if (Math.pow(right_trigger, 3) > 0) {
            right_trigger = Math.max(Math.pow(right_trigger, 3), 0.7);
        } else if (Math.pow(right_trigger, 3) < 0) {
            right_trigger = Math.min(Math.pow(right_trigger, 3), -0.7);
        } else {
            right_trigger = 0;
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
            left_trigger = 0;
            right_trigger = 0;
            driveFor(0.5, 1.5);
        }

        motor_drive_left.setPower(left_trigger);
        motor_drive_right.setPower(right_trigger);

        startPrev = startButton;
        telemetry.addData("Start Button", startButton);
        telemetry.addData("Start Previous", startPrev);
    }

    public void driveFor(double power, double seconds) {
        motor_drive_right.setPower(power);
        motor_drive_left.setPower(power);

        seconds = seconds * 1000;

        instant = runtime.milliseconds();
        while (instant > runtime.milliseconds() - seconds) {
            telemetry.addData("Time Left", (seconds - (runtime.milliseconds() - instant)));
        }

        motor_drive_left.setPower(0);
        motor_drive_right.setPower(0);
    }

}