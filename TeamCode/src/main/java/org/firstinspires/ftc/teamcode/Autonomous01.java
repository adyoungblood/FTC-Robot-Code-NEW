/*
*/
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.text.SimpleDateFormat;
import java.util.Date;

@Autonomous(name="Autonomous01", group="Linear Opmode")  // @TeleOp(...) is the other common choice

public class Autonomous01 extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor intake_motor_1;

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

    public double left_train_power;
    public double right_train_power;

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");

        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");
        //smol test comment
        telemetry.addData("Status", "Running: " + runtime.toString());

        while (opModeIsActive()) {
                driveFor(1, 1, 1500);
                driveFor(-1, 1, 3000);
                driveFor(1, 1, 1500);
                driveFor(-1, 1, 3000);
                driveFor(1, 1, 1500);
                driveFor(-1, 1, 3000);
                driveFor(1, 1, 1500);
                driveFor(-1, 1, 3000);
                driveFor(0, 0, 1000);
            telemetry.addData("OpMode complete", "");
            break;
        }
    }

    public void driveFor(int left_train_power, int right_train_power, int milliseconds) {
        motor_drive_left.setPower(left_train_power);
        motor_drive_right.setPower(-right_train_power);
        try {
            Thread.sleep(milliseconds);
            /*
            if (gamepad1.start) {
                Thread.currentThread().interrupt();
            }
            */
            // Somehow make this check no matter what is happening
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}