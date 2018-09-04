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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="OVSStrong_Blue_Auto", group="OVSStrong")
public class OVSStrong_Blue_Auto extends LinearOpMode {

    /* Declare OpMode members. */
    OVSStrongHardware robot = new OVSStrongHardware();
    private ElapsedTime runtime = new ElapsedTime();
    private boolean sensedRed(){
        boolean lColorRed = robot.lColor.red() >= 20 && robot.lColor.blue() < robot.lColor.red(),
                rColorRed = robot.rColor.red() >= 20 && robot.rColor.blue() < robot.rColor.red();
        return lColorRed || rColorRed;
    }
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("OVSStrong", "We Will Rebuild!");
        telemetry.update();
        runtime.reset();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int counter = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            switch(counter){
                case 0:
                    robot.lowerJewelKicker();
                    counter++;
                    break;
                case 1:
                    sleep(2000);
                    if(robot.color.blue() > robot.color.red()) counter += 2;
                    else if(robot.color.red() > robot.color.blue()) counter++;
                    break;
                case 2:
                    // go backward than go forward a little
                    robot.move(Math.PI / 2, 0.4, 0, OVSStrongHardware.TurnDirection.NONE);
                    sleep(400);
                    robot.retractJewelKicker();
                    robot.move(3 * Math.PI / 2, 0.4, 0, OVSStrongHardware.TurnDirection.NONE);
                    sleep(400);
                    robot.stop();
                    sleep(1000);
                    counter += 2;
                    break;
                case 3:
                    // go forward a little for 0.5s
                    robot.move(3 * Math.PI / 2, 0.4, 0, OVSStrongHardware.TurnDirection.NONE);
                    sleep(400);
                    robot.retractJewelKicker();
                    robot.stop();
                    sleep(1000);
                    // counter++;
                    counter += 2;
                    break;
                // case 4:
                //     robot.move(Math.PI / 2, 0.4, 0, OVSStrongHardware.TurnDirection.NONE);
                //     sleep(1500);
                //     counter++;
                //     break;
                // case 5:
                //     if(sensedRed()) {
                //         robot.stop();
                //         counter++;
                //     }
                //     break;
                case 4:
                    robot.move(3 * Math.PI / 2, 0.6, 0, OVSStrongHardware.TurnDirection.NONE);
                    sleep(1200);
                    robot.stop();
                    counter += 2;
                    break;
                case 5:
                    robot.move(3 * Math.PI / 2, 0.6, 0, OVSStrongHardware.TurnDirection.NONE);
                    sleep(800);
                    robot.stop();
                    counter++;
                    break;
                default:
                    telemetry.addData("Status", "Auto Blue Completed. \n#OVSStrong");
                    telemetry.update();
                    break;
            }
            telemetry.addData("counter", 5);
        }
    }
}
