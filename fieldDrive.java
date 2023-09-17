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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp()
public class fieldDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private OrientationSensor imu;

    public double leftStickY;
    public double leftStickX;
    public double rightStickX;
    public double FL_power_raw, FR_power_raw, RL_power_raw, RR_power_raw;
    public double FL_power, FR_power, RL_power, RR_power;

    public double newForward;
    public double newStrafe;

    public Orientation angles;

    @Override
    public void init() {
        imu = hardwareMap.get(OrientationSensor.class, "imu");
        frontLeft  = hardwareMap.get(DcMotor.class, "FL_wheel");
        frontRight = hardwareMap.get(DcMotor.class, "FR_wheel");
        backLeft  = hardwareMap.get(DcMotor.class, "BL_wheel");
        backRight = hardwareMap.get(DcMotor.class, "BR_wheel");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        controls();
        // Show the elapsed game time and wheel power.
        telemetries();
    }

    @Override
    public void stop() {
    }
    public void controls()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        driveFormula();
        setPower();
    }

    /*public void getJoyValues()
    {
        leftStickY = gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        float pi = 3.1415926f;

        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi/180;
        newForward = leftStickY * Math.cos(gyro_radians) + leftStickX * Math.sin(gyro_radians);
        newStrafe = -leftStickY * Math.sin(gyro_radians) + leftStickX * Math.cos(gyro_radians);
    }
*/
    public void driveFormula()
    {
      double r = Math.hypot(leftStickX, leftStickY);
      double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
      double rightX = rightStickX;
      final double v1 = r * Math.cos(robotAngle) + rightX;
      final double v2 = r * Math.sin(robotAngle) - rightX;
      final double v3 = r * Math.sin(robotAngle) + rightX;
      final double v4 = r * Math.cos(robotAngle) - rightX;
    }

    public void setPower()
    {
        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);
    }
    public void telemetries(){
        telemetry.addData("Gamepad LY, LX, RX", gamepad1.left_stick_y + " " + gamepad1.left_stick_x + " " + gamepad1.right_stick_x);
        telemetry.addData("Motors", "FL (%.2f), FR (%.2f) BL (%.2f), BR (%.2f)", FL_power, FR_power, RL_power, RR_power);
        telemetry.update();
    }
}




}
