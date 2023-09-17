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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp()
public class robotDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public double leftStickY;
    public double leftStickX;
    public double rightStickX;
    public double FL_power_raw, FR_power_raw, RL_power_raw, RR_power_raw;
    public double FL_power, FR_power, RL_power, RR_power;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "FL_wheel");
        frontRight = hardwareMap.get(DcMotor.class, "FR_wheel");
        backLeft  = hardwareMap.get(DcMotor.class, "BL_wheel");
        backRight = hardwareMap.get(DcMotor.class, "BR_wheel");

        // Most robots need the motor on one side to be reversed to drive forward

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive();
        // Show the elapsed game time and wheel power.
        telemetries();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    public void drive()
    {

        leftStickY = gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        FL_power_raw = leftStickY - leftStickX - rightStickX;
        FR_power_raw = leftStickY + leftStickX + rightStickX;
        RL_power_raw = leftStickY + leftStickX - rightStickX;
        RR_power_raw = leftStickY - leftStickX + rightStickX;

        FL_power = Range.clip(FL_power_raw, -1, 1);
        FR_power = Range.clip(FR_power_raw, -1, 1);
        RL_power = Range.clip(RL_power_raw,-1 ,1);
        RR_power = Range.clip(RR_power_raw, -1, 1);

        frontLeft.setPower(FL_power);
        frontRight.setPower(FR_power);
        backLeft.setPower(RL_power);
        backRight.setPower(RR_power);
    }


    public void telemetries(){
        telemetry.addData("Controller LY, LX, RX", gamepad1.left_stick_y + " " + gamepad1.left_stick_x + " " + gamepad1.right_stick_x);
        telemetry.addData("Motors", "FL (%.2f), FR (%.2f) BL (%.2f), BR (%.2f)", FL_power, FR_power, RL_power, RR_power);
        telemetry.update();
    }
}
