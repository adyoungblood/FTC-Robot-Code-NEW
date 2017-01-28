/*
*/
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

//import java.text.SimpleDateFormat;
//import java.util.Date;
@Disabled
@TeleOp(name="ShooterTest", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class ShooterTest extends OpMode {
    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor intake_motor_1;

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


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        shooter_motor_1 = hardwareMap.dcMotor.get("Motor_Shooter_1");
//        intake_motor_1 = hardwareMap.dcMotor.get("Motor_Intake_1");

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

        shooter_control();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void shooter_control() {

/*
        if (gamepad1.right_stick_y > 0.2) {
            shooter_motor_1.setPower(1);
        }
        if (gamepad1.right_stick_y < -0.2) {
            shooter_motor_1.setPower(-0.3);
        }

        if (gamepad1.right_stick_y >= -0.2 && gamepad1.right_stick_y <= 0.2) {
            shooter_motor_1.setPower(0);
        }
*/

        if (gamepad1.right_trigger > 0.2) {
            shooter_motor_1.setPower(-0.4);
            try {
                TimeUnit.MILLISECONDS.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            shooter_motor_1.setPower(0);

            try {
                TimeUnit.MILLISECONDS.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            shooter_motor_1.setPower(1);

            try {
                TimeUnit.MILLISECONDS.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            shooter_motor_1.setPower(0);
        }
        shooter_motor_1.setPower(0);



/*        if (joystick1_right_y > 0.3) {
            intake_motor_1.setPower(-joystick2_right_y);
        }
        if (joystick1_right_y < -0.3) {
            intake_motor_1.setPower(0.15);
        }

        if (joystick1_right_y >= -0.3 && joystick1_right_y <= 0.3) {
            intake_motor_1.setPower(0);
        }


        if (gamepad2.start) {
            intake_motor_1.setPower(0);
        }
        */
    }

}