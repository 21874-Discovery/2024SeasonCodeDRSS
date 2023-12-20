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

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="RoboBobo", group="Robot")
public class RoboBoboAuto extends LinearOpMode {

    // declare OpMode members

    private int step=1;

    private DcMotor topLeft;
    private DcMotor topRight;
    private DcMotor bottomLeft;
    private DcMotor bottomRight;

    private double topLeftCount;
    private double topRightCount;
    private double bottomLeftCount;
    private double bottomRightCount;


    private final double ppr=537.7;
    private final double     dgr    = 1.0 ;     // No External Gearing.
    private final double     wd   = 3.77953 ;     // For figuring circumference
    private final double     tpi         = (ppr * dgr)/(wd * Math.PI);

    @Override
    public void runOpMode() {

        topLeft=hardwareMap.get(DcMotor.class,"topLeft");
        topRight=hardwareMap.get(DcMotor.class,"topRight");
        bottomLeft=hardwareMap.get(DcMotor.class,"bottomLeft");
        bottomRight=hardwareMap.get(DcMotor.class,"bottomRight");

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //zero power behav

        //set position of wanted inches

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setTargetPosition((int)(29*tpi));
        topRight.setTargetPosition((int)(29*tpi));
        bottomLeft.setTargetPosition((int)(29*tpi));
        bottomRight.setTargetPosition((int)(29*tpi));


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // init intake motors
        // init servo

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            /* READ INPUTS */
            // read current values of joystick
            topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            topLeft.setPower(dashboardData.drivePower);
            topRight.setPower(dashboardData.drivePower);
            bottomLeft.setPower(dashboardData.drivePower);
            bottomRight.setPower(dashboardData.drivePower);

            while(step==1){
                if (topLeft.getCurrentPosition() >= Math.abs(topLeft.getTargetPosition())) {
                    //keep moving robot until 4 revolutions
                    step = 2;
                    topLeft.setPower(0);
                    topRight.setPower(0);
                    bottomLeft.setPower(0);
                    bottomRight.setPower(0);


                    topLeftCount = topLeft.getCurrentPosition();
                    topRightCount = topRight.getCurrentPosition();
                    bottomLeftCount = bottomLeft.getCurrentPosition();
                    bottomRightCount = bottomRight.getCurrentPosition();

                    topLeft.setTargetPosition((int)(17.28*tpi));
                    topRight.setTargetPosition((int)(17.28*tpi));
                    bottomLeft.setTargetPosition((int)(17.28*tpi));
                    bottomRight.setTargetPosition((int)(17.28*tpi));

                    topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                    bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }
            topLeft.setPower(dashboardData.drivePower);
            topRight.setPower(dashboardData.drivePower);
            bottomLeft.setPower(dashboardData.drivePower);
            bottomRight.setPower(dashboardData.drivePower);
            while(step==2){
                if (topLeft.getCurrentPosition() >= Math.abs(topLeft.getTargetPosition())) {
                    //keep moving robot until 4 revolutions
                    step = 3;
                    topLeft.setPower(0);
                    topRight.setPower(0);
                    bottomLeft.setPower(0);
                    bottomRight.setPower(0);


                    topLeftCount = topLeft.getCurrentPosition();
                    topRightCount = topRight.getCurrentPosition();
                    bottomLeftCount = bottomLeft.getCurrentPosition();
                    bottomRightCount = bottomRight.getCurrentPosition();

                    topLeft.setTargetPosition((int)(48*tpi));
                    topRight.setTargetPosition((int)(48*tpi));
                    bottomLeft.setTargetPosition((int)(48*tpi));
                    bottomRight.setTargetPosition((int)(48*tpi));


                    topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                    bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }
            while(step==3){
                topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            telemetry.addData("Status", "Initialized");
            telemetry.update();

        }
        topLeft.setPower(0.0);
        topRight.setPower(0.0);
        bottomLeft.setPower(0.0);
        bottomRight.setPower(0.0);
    }
}