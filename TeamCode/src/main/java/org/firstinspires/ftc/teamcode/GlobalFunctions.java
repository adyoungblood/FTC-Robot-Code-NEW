package org.firstinspires.ftc.teamcode;

import android.content.Context;
//import android.media.SoundPool;

//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import java.math.*;

public abstract class GlobalFunctions extends LinearOpMode {
    Context mContext;

    // constructor
    public GlobalFunctions(Context context){
        this.mContext = context;
    }

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotorController motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
    public DcMotor motor_drive_left;
    public DcMotor motor_drive_right;
    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor shooter_motor_2;
    public DcMotor intake_motor;
    public DcMotor belt_motor;
    //public Servo buttonServo;
    public DcMotorController motor_controller_other;
    //public DcMotor motor_hat;

    // Initilization of drive train variables:
    //public double power_forward;
    //public double power_back;
    //    public double power_RT;
//    public double power_LT;
    //public double power_level;

    // Initilization of drive train variables:
    //public double power_forward;
    //public double power_shooter;

    // Initialization of joystick buttons:
    //public double button_RT;
    //public double button_LT;

    public boolean button_RB;
    public boolean button_LB;
    //public boolean button_a;
    //public boolean button_y;
    public boolean button_b;
    //public boolean button_x;

    //public double joystick1_right_x;
    //public double joystick1_right_y;
    //public double joystick1_left_x;
    //public double joystick1_left_y;
    //public double joystick1_right_x;
    //public double joystick2_right_y;

    //public double ARM_RETRACTED_POSITION = 0.2;
    //public double ARM_EXTENDED_POSITION = 0.8;

    //public double left_train_power;
    //public double right_train_power;

    public boolean startButton;
    public boolean startPrev;
    public boolean speed;

    //public SoundPool mySound;
    //public int beepID;

    double left_trigger;
    double right_trigger;

    //ColorSensor sensorRGB;
    //DeviceInterfaceModule cdim;

    double instant;

    public void driveFor(double seconds, double left_power, double right_power) {
        motor_drive_right.setPower(right_power);
        motor_drive_left.setPower(left_power);

        //1 second = 50 inches
        waitFor(seconds);

        motor_drive_left.setPower(0);
        motor_drive_right.setPower(0);
    }

    public void driveFor(double seconds, double power) {
        driveFor(seconds, power, power);
    }

    public void driveFor(double seconds) {
        driveFor(seconds, 0.75, 0.75);
    }

    public void waitFor(double seconds) {
        seconds = seconds * 1000;

        instant = runtime.milliseconds();
        while (instant > runtime.milliseconds() - seconds) {
            telemetry.addData("Time Left", (seconds - (runtime.milliseconds() - instant)));
        }
    }
    //Drives for a random power, and for a random time.
    //Directional input is accepted, but it defaults to forwards

    /*
    public void driveRandomAF(String direction) {
        if (direction == "forward") {
            driveFor(Math.random(), Math.random(), Math.random() * 2);
        } else if (direction == "left") {
            driveFor(-Math.random(), Math.random(), Math.random() * 2);
        } else if (direction == "right") {
            driveFor(Math.random(), -Math.random(), Math.random() * 2);
        } else if (direction == "backward") {
            driveFor(-Math.random(), -Math.random(), Math.random() * 2);
        } else {
            driveFor(Math.random(), Math.random(), Math.random() * 2);
        }
    }
    */

}