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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="OVSStrong_Control", group="OVSStrong")
public class OVSStrong_Control extends LinearOpMode {

    /* Declare OpMode members. */
    OVSStrongHardware robot = new OVSStrongHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("OVSStrong", "We Will Rebuild!");
        telemetry.update();
        runtime.reset();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            robot.move(
                    Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x),
                    Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y),
                    Math.max(gamepad1.left_trigger, gamepad1.right_trigger),
                    gamepad1.left_trigger < gamepad1.right_trigger ? OVSStrongHardware.TurnDirection.RIGHT : OVSStrongHardware.TurnDirection.LEFT);
            // telemetry.addData("lf", robot.lfDrive.getPower());
            // telemetry.addData("rf", robot.rfDrive.getPower());
            // telemetry.addData("lb", robot.lbDrive.getPower());
            // telemetry.addData("rb", robot.rbDrive.getPower());
            if(gamepad1.left_bumper){
                robot.lfServo.setPosition(1.0);
                robot.rfServo.setPosition(0.5);
            } else if(gamepad1.right_bumper) {
                robot.lfServo.setPosition(0.5);
                robot.rfServo.setPosition(1.0);
            }

            robot.parallex.setPower(-gamepad1.right_stick_y / 2);

            if(gamepad1.dpad_right) robot.relicExtender.setPower(0.5);
            else if(gamepad1.dpad_left) robot.relicExtender.setPower(-0.5);
            else robot.relicExtender.setPower(0);

            if(gamepad1.x) robot.hand.setPosition(0.47);
            else if(gamepad1.b) robot.hand.setPosition(1);

            if(gamepad1.dpad_up) robot.wrist.setPosition(0.8);
            else if(gamepad1.dpad_down) robot.wrist.setPosition(0.2);
            else robot.wrist.setPosition(0.5);

            telemetry.addData("red", robot.rColor.red());
            telemetry.addData("green", robot.rColor.green());
            telemetry.addData("blue", robot.rColor.blue());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
