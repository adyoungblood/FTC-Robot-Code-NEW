package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/* (V 2.0 of drivetrain program; incl alexanders suggestions (thx alexander); so partially pressing down a button will
 * also make the servos go partially.
 * Basic program for Victanus's new drive train.
 * Basics:
 *  Forward - gamepad 2a, 1a = 1 (pressed) Motors left, right = 1
 * Turn right  - gamepad 2a, 1b = 1 (pressed) Motor right = -1, motor left = 1
 * Turn left  -  gamepad 2b, 1a = 1 (pressed) Motor right = 1, motor left = -1
 * Backward - gampad 2b, 1b = 1 (pressed) Motor right = -1, motor left = -1
*  else - Motors left, right = 0
 */
@TeleOp(name = "Drive02", group = "Tutorials")
class Drive02 extends LinearOpMode
{
    // Declare drive motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        // If drive motors are given full power, robot would spin because of the motors being in
        // opposite directions. So reverse one
        motorLeft.setDirection(DcMotor.Direction.REVERSE);


        // Wait until start button is pressed
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        while(opModeIsActive())
        {
            // Move drivetrain if requested
            if (gamepad1.a && gamepad2.a)
            {
                motorLeft.setPower(gamepad1.a);
                // Both set to same button to prevent the veering of motors; and to ensure robot actually goes straight
                motorRight.setPower(gamepad1.a);
            }
            if(gamepad1.b && gamepad2.b)
            {
                // Both set to same button to prevent the veering of motors; and to ensure robot actually goes straight
                motorLeft.setPower(-gamepad2.b);
                motorRight.setPower(-gamepad2.b);
            }
            if(gamepad1.a && gamepad2.b)
            {
                // change done to previous ones not done to this, due to the fact that steering needs max. manueverability
                motorLeft.setPower(gamepad1.a);
                motorRight.setPower(-gamepad2.b);
            }
            if(gamepad1.b && gamepad2.a)
            {
                motorLeft.setPower(-gamepad1.b);
                motorRight.setPower(gamepad2.a);
            }
            else
                motorLeft.setPower(0);
            motorRight.setPower(0);
            // Give hardware a chance to catch up
            idle();
        }
    }
}
