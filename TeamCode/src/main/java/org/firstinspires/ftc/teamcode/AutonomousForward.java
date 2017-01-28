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

    GlobalFunctions myGlog;

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorController motor_controller_drive;
    public DcMotor motor_drive_left;
    public DcMotor motor_drive_right;

    double instant;
    int x = 0;

    @Override
    public void runOpMode() {
        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");
        motor_drive_left.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_right.setDirection(DcMotorSimple.Direction.FORWARD);


        while (opModeIsActive()) {
            if (x == 0) {

                //1 second is about 50 inches @ 75% power
                myGlog.driveFor(2);

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