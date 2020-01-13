package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Linear Opmode")
public class TeleOp extends LinearOpMode {

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {

        robot.teleInit(hardwareMap);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;

            double rx = gamepad1.right_stick_x;
            double ry = -gamepad1.right_stick_y;

            double s = .8;

            // Send calculated power to wheels
            robot.frontLeft.setPower  (s * Range.clip(ly + rx + lx, -1.0, 1.0));
            robot.backLeft.setPower   (s * Range.clip(ly + rx - lx, -1.0, 1.0));
            robot.frontRight.setPower (s * Range.clip(ly - rx - lx, -1.0, 1.0));
            robot.backRight.setPower  (s * Range.clip(ly - rx + lx, -1.0, 1.0));

            //Claw
            if (gamepad2.y) {
                robot.claw.setPosition(0.5);
            }
            else if (gamepad2.a) {
                robot.claw.setPosition(0.85);
            }

            //Wrist
            if (gamepad2.dpad_right) {
                robot.wrist.setPower(0.5);
            }
            else if (gamepad2.dpad_left) {
                robot.wrist.setPower(-0.5);
            }
            else {
                robot.wrist.setPower(0);
            }

            //Arm
            robot.arm.setPower(gamepad2.left_stick_y * 0.6);

            /*//Lift
            if (gamepad2.dpad_up) {
                robot.lift.setPower(0.7);
            }
            else if (gamepad2.dpad_down) {
                robot.lift.setPower(-0.7);
            }
            else {
                robot.lift.setPower(0);
            }*/


        }
    }
}
