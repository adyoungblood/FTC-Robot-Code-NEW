package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Autonomous04Blue extends LinearOpMode {

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

    double instant;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        // Initialize drive motors
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

        //TODO This is supposed to be the new Autonomous with shooting and cool stuff
        //TODO I need measurements but I should do it now anyway but again I'm lazy xD
        //driveFor(0.5, 1);
        shoot();


    }

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

    public void shoot() {
        intake_servo.setPosition(0);
        shooter_motor_1.setPower(0.3);
        shooter_motor_2.setPower(0.3);
        waitFor(2);
        belt_motor.setPower(1);
        waitFor(0.5);
        intake_servo.setPosition(0.05);
        waitFor(2);
        intake_servo.setPosition(0);
        shooter_motor_1.setPower(0);
        shooter_motor_2.setPower(0);
        belt_motor.setPower(0);
    }

}
