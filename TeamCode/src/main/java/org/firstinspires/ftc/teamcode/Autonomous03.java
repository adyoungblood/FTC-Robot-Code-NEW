package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;


@Autonomous(name="Autonomous03", group="Iterative Opmode")  // @TeleOp(...) is the other common choice

public class Autonomous03 extends LinearOpMode {

    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor intake_motor_1;
    public DcMotor intake_motor_2;

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

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");

        //motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        //motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        //shooter_motor_1 = hardwareMap.dcMotor.get("Motor_Shooter_1");
        //intake_motor_1 = hardwareMap.dcMotor.get("Motor_Intake_1");
        //intake_motor_2 = hardwareMap.dcMotor.get("Motor_Intake_2");
        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");

        motor_drive_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_drive_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_drive_left.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_drive_right.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_drive_left.setTargetPosition(0);
        motor_drive_right.setTargetPosition(0);

        drive(1);
        }



    public void drive(int distance) {
        motor_drive_left.setTargetPosition(distance);
        motor_drive_right.setTargetPosition(distance);
        // TODO Test how many turns it takes to go how many inches
    }
/*
    public void setDriveMode(DcMotorController.RunMode mode) {
        if (motor_drive_left.getChannelMode() != mode) {
            motor_drive_right.setChannelMode(mode);
        }

        if (motor_drive_right.getChannelMode() != mode) {
            motor_drive_right.setChannelMode(mode);
        }
    }

    public void setDrivePower(double left, double right) {
        // This assumes power is given as -1 to 1
        // The range clip will make sure it is between -1 and 1
        // An incorrect value can cause the program to exception
        motor_drive_left.setPower(Range.clip(left, -1.0, 1.0));
        motor_drive_right.setPower(Range.clip(right, -1.0, 1.0));
    }
    */
}