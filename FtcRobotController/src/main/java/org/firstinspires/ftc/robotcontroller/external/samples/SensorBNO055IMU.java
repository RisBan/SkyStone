/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "Sensor: BNO055 IMU", group = "Sensor") // Comment this out to add to the opmode list
public class SensorBNO055IMU extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Orientation lastAngles = new Orientation();
    Acceleration gravity;

    double globalAngle;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        DcMotor frontRight = null;
        DcMotor frontLeft = null;
        DcMotor backLeft = null;
        DcMotor backRight = null;

    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();

            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;

            double rx = gamepad1.right_stick_x;
            double ry = -gamepad1.right_stick_y;


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            /*
            fl = ly + rx + lx
            bl = ly + rx - lx
            fr = ly - rx - lx
            br = ly - rx + lx
             */

            double s = .5;

            // Send calculated power to wheels
            frontLeft.setPower  (s * Range.clip(ly + rx + lx, -1.0, 1.0));
            backLeft.setPower   (s * Range.clip(ly + rx - lx, -1.0, 1.0));
            frontRight.setPower (s * Range.clip(ly - rx - lx, -1.0, 1.0));
            backRight.setPower  (s * Range.clip(ly - rx + lx, -1.0, 1.0));

            if (gamepad1.a) {
                rotateRobot(90,0.5);
            }


            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", motorPower, rightPower);
            telemetry.update();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

//    void composeTelemetry() {
//
//        // At the beginning of each telemetry update, grab a bunch of data
//        // from the IMU that we will then display in separate lines.
//        telemetry.addAction(new Runnable() { @Override public void run()
//                {
//                // Acquiring the angles is relatively expensive; we don't want
//                // to do that in each of the three items that need that info, as that's
//                // three times the necessary expense.
//                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity  = imu.getGravity();
//                }
//            });
//
//        telemetry.addLine()
//            .addData("status", new Func<String>() {
//                @Override public String value() {
//                    return imu.getSystemStatus().toShortString();
//                    }
//                })
//            .addData("calib", new Func<String>() {
//                @Override public String value() {
//                    return imu.getCalibrationStatus().toString();
//                    }
//                });
//
//        telemetry.addLine()
//            .addData("heading", new Func<String>() {
//                @Override public String value() {
//                    return formatAngle(angles.angleUnit, angles.firstAngle);
//                    }
//                })
//            .addData("roll", new Func<String>() {
//                @Override public String value() {
//                    return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//            .addData("pitch", new Func<String>() {
//                @Override public String value() {
//                    return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//            .addData("grvty", new Func<String>() {
//                @Override public String value() {
//                    return gravity.toString();
//                    }
//                })
//            .addData("mag", new Func<String>() {
//                @Override public String value() {
//                    return String.format(Locale.getDefault(), "%.3f",
//                            Math.sqrt(gravity.xAccel*gravity.xAccel
//                                    + gravity.yAccel*gravity.yAccel
//                                    + gravity.zAccel*gravity.zAccel));
//                    }
//                });
//    }
//
//    //----------------------------------------------------------------------------------------------
//    // Formatting
//    //----------------------------------------------------------------------------------------------
//
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
//
//    String formatDegrees(double degrees){
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }

    private void resetAngle()
    {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
    }

    private double getAngle()
        {
            // We experimentally determined the Z axis is the axis we want to use for heading angle.
            // We have to process the angle because the imu works in euler angles so the Z axis is
            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            globalAngle += deltaAngle;

            lastAngles = angles;

            return globalAngle;
        }

    void rotateRobot(int degrees, double power)
        {
            double  leftPower, rightPower;

            // restart imu movement tracking.
            resetAngle();

            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).

            if (degrees < 0)
            {   // turn right.
                leftPower = power;
                rightPower = -power;
            }
            else if (degrees > 0)
            {   // turn left.
                leftPower = -power;
                rightPower = power;
            }
            else return;

            // set power to rotate.
            backLeft.setPower(leftPower);
            frontLeft.setPower(leftPower);
            backRight.setPower(rightPower);
            frontRight.setPower(rightPower);

            // rotate until turn is completed.
            if (degrees < 0)
            {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0) {}

                while (opModeIsActive() && getAngle() > degrees) {}
            }
            else    // left turn.
                while (opModeIsActive() && getAngle() < degrees) {}

            // turn the motors off.
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // wait for rotation to stop.
            sleep(1000);

            // reset angle tracking on new heading.
            resetAngle();
        }
}
