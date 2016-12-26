/*
*/
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import java.text.SimpleDateFormat;
//import java.util.Date;

@TeleOp(name="Drive02", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class Drive02 extends OpMode {
    /* Initialize Variables */
    private ElapsedTime runtime = new ElapsedTime();

    /* Initialize Motors */
    public DcMotorController motor_controller_left;
    public DcMotorController motor_controller_right;
    public DcMotor left_front_motor;
    public DcMotor right_front_motor;
    public DcMotor left_back_motor;
    public DcMotor right_back_motor;

    // Initilization of drive train variables:
    public double power_back;
    public double power_forward;

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
//    public double joystick2_right_y;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        //motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        motor_controller_left = hardwareMap.dcMotorController.get("Motor_Controller_Left");
        motor_controller_right = hardwareMap.dcMotorController.get("Motor_Controller_Right");

        left_front_motor = hardwareMap.dcMotor.get("Motor_Left_Front");
        left_back_motor = hardwareMap.dcMotor.get("Motor_Left_Back");
        right_front_motor = hardwareMap.dcMotor.get("Motor_Right_Front");
        right_back_motor = hardwareMap.dcMotor.get("Motor_Right_Back");

        power_forward = 1;
        power_back = 0.5;

//        telemetry.addData("ServoPosition", "Position: " + steering_servo.getPosition());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {

        //runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        drive_control();

//        telemetry.addData("Status", "Running: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


//******************************************************************************
//------------------------------------------------------------------------------
//Routine for Driving Robot with Front Buttons:

    public void drive_control() {

        // Define mode in which power is controlled:
        // Control with triggers

        button_RT = gamepad1.right_trigger * power_forward;
        button_RB = gamepad1.right_bumper;

        if (button_RT > 0) {
            right_front_motor.setPower(button_RT);
            right_back_motor.setPower(button_RT);
        } else if (button_RB) {
            right_front_motor.setPower(-power_back);
            right_back_motor.setPower(-power_back);
        } else {
//            motor_drive_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coasting
            right_front_motor.setPower(0);
            right_back_motor.setPower(0);
        }

        button_LT = gamepad1.left_trigger * power_forward;
        button_LB = gamepad1.left_bumper;

        if (button_LT > 0) {
            left_front_motor.setPower(-button_LT);
            left_back_motor.setPower(-button_LT);
        } else if (button_LB) {
            left_front_motor.setPower(power_back);
            left_back_motor.setPower(power_back);
        } else {
//            motor_drive_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coasting
            left_front_motor.setPower(0);
            left_back_motor.setPower(0);
        }

        // Joystick control
/*
        joystick1_left_y = gamepad1.left_stick_y * power_forward;

        if(gamepad1.left_bumper) {
            power_forward = 0.25;

        }
        if(gamepad1.right_bumper) {
            power_forward = 1;

        }


        if (joystick1_left_y > 0.2) {
            left_front_motor.setPower(-joystick1_left_y);
            right_front_motor.setPower(joystick1_left_y);
            left_back_motor.setPower(-joystick1_left_y);
            right_back_motor.setPower(joystick1_left_y);
        }
        if (joystick1_left_y < -0.2) {
            left_front_motor.setPower(-joystick1_left_y);
            right_front_motor.setPower(joystick1_left_y);
            left_back_motor.setPower(-joystick1_left_y);
            right_back_motor.setPower(joystick1_left_y);
        }

        if (joystick1_left_y >= -0.2 && joystick1_left_y <= 0.2) {
            left_front_motor.setPower(0);
            right_front_motor.setPower(0);
            left_back_motor.setPower(0);
            right_back_motor.setPower(0);
        }

    }
*/

    }

}