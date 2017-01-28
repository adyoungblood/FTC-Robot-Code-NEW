package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;




@Autonomous
public class AutonomousForward extends LinearOpMode {

    GlobalFunctions g;

    double instant;
    int x = 0;

    @Override
    public void runOpMode() {
        g.motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        g.motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        g.motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");
        g.motor_drive_left.setDirection(DcMotorSimple.Direction.REVERSE);
        g.motor_drive_right.setDirection(DcMotorSimple.Direction.FORWARD);


        while (opModeIsActive()) {
            if (x == 0) {

                //1 second is about 50 inches @ 75% power
                g.driveFor(2);

                x++;
            } else {
                telemetry.addData("Status", "Autonomous done");
                requestOpModeStop();
                resetStartTime();
                stop();
                gamepad1.reset();
                gamepad2.reset();
            }
        }
    }
}