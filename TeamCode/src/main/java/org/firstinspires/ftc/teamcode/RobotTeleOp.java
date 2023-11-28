/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (sconditions are met:
 *
 * Redistriubject to the limitations in the disclaimer below) provided that
 * the following butions of source code must retain the above copyright notice, this list
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


@TeleOp(name="RobotTeleOp", group="Linear Opmode")
public class RobotTeleOp extends LinearOpMode {

   // declare OpMode members
   // drivetrain
   private DcMotor         frontLeft           = null;
   private DcMotor         frontRight          = null;
   private DcMotor         backLeft            = null;
   private DcMotor         backRight           = null;

   // intake
   private DcMotor         intakeMotor        = null;

   // arm
   private DcMotor         armMotor            = null;

   private Servo           clawServo           = null;

   // power for intake (can be changed later)
   private double          intakePower         = 0.5;

   // toggles so that only one input is registered per button press
   boolean dirToggle=true,clawToggle=true;

   @Override
   public void runOpMode() {

      telemetry.addData("Status", "Initialized");
      telemetry.update();

      // init the drive system variables
      frontLeft       = hardwareMap.get(DcMotor.class, "frontLeft");
      frontRight      = hardwareMap.get(DcMotor.class, "frontRight");
      backLeft        = hardwareMap.get(DcMotor.class, "backLeft");
      backRight       = hardwareMap.get(DcMotor.class, "backRight");

      // init intake motors
      intakeMotor    = hardwareMap.get(DcMotor.class, "intakeMotor");


      // init arm motor
      armMotor        = hardwareMap.get(DcMotor.class, "armMotor");

      // init servo
      clawServo       = hardwareMap.get(Servo.class, "clawServo");

      // left is reverse of right
      frontLeft.setDirection(DcMotor.Direction.REVERSE);
      frontRight.setDirection(DcMotor.Direction.FORWARD);
      backLeft.setDirection(DcMotor.Direction.REVERSE);
      backRight.setDirection(DcMotor.Direction.FORWARD);

      // TODO: may have to flip these
      intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

      //arm direction
      armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

      //set default position of servo as 0
      clawServo.setPosition(0);

      // Wait for the game to start (driver presses PLAY)
      waitForStart();

      // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {
         /* READ INPUTS */
         // read current values of joystick
         float lx    = gamepad1.left_stick_x;
         float ly    = -gamepad1.left_stick_y;
         float rx    = gamepad1.right_stick_x;
         float ry    = -gamepad2.right_stick_y;

         // read current value of right bumper
         boolean rBumper = gamepad1.right_bumper;
         boolean lBumper = gamepad1.left_bumper;

         // read the face buttons
         boolean a = gamepad2.a;


         /* DO MOTOR STUFF */
         // add the abs of all joystick inputs
         // get the max of ^ and 1.0
         // used to ensure controller inputs are proportionally applied to motor outputs
         float max   = Math.max( Math.abs(lx)+Math.abs(ly)+Math.abs(rx) , 1.0f);

         // normalize each input by either the sum of all 3 joysticks or 1.0f
         float nlx   = lx / max;
         float nly   = ly / max;
         float nrx   = rx / max;

         // apply mechanum drive equations to
         // create commands for each drivetrain motor
         double fl   = nlx   +nly    +nrx;
         double bl   = -nlx  +nly    +nrx;
         double fr   = -nlx  +nly    -nrx;
         double br   = nlx   +nly    -nrx;

         // give motor commands
         frontLeft.setPower(fl);
         backLeft.setPower(bl);
         frontRight.setPower(fr);
         backRight.setPower(br);


         /* DO INTAKE STUFF */
         if (rBumper) {
            intakeMotor.setPower(intakePower);
         }
         else {
            intakeMotor.setPower(0);
         }
         if (lBumper) {
            //only lets the code below run once
            if(dirToggle) {
               //swaps the direction of the motors, therefore reversing the direction
               if(intakeMotor.getDirection()==DcMotorSimple.Direction.FORWARD){
                  intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
               }
               else if(intakeMotor.getDirection()==DcMotorSimple.Direction.REVERSE){
                  intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
               }
               //disables further presses
               dirToggle = false;
            }
         }
         else{

            //when button is released, let new button presses be registered
            dirToggle=true;
         }
         if(a){
            if(clawToggle) {
               if(clawServo.getPosition()==0||clawServo.getPosition()==-1) {
                  clawServo.setPosition(1);
               }
               else{
                  clawServo.setPosition(-1);
               }
               clawToggle=false;
            }
         }
         else{
            clawToggle=true;
         }

         // set arm power
         armMotor.setPower(ry);

         /* PRINT STUFF */


         // print raw joystick readings
         telemetry.addData("lx: ", lx);
         telemetry.addData("ly: ", ly);
         telemetry.addData("rx: ", rx);

         // print the result of the max function
         telemetry.addData("max: ", max);

         // print normalized joystick readings
         telemetry.addData("nlx: ", nlx);
         telemetry.addData("nly: ", nly);
         telemetry.addData("nrx: ", nrx);

         // print motor command values
         telemetry.addData("fl: ", fl);
         telemetry.addData("bl: ", bl);
         telemetry.addData("fr: ", fr);
         //wrote "New" so we can know if it's the newest version
         telemetry.addData("br: New", br);

         // TODO: print time the loop took to execute

         telemetry.update();
      }

      // stop all the motors
      frontLeft.setPower(0.0);
      frontRight.setPower(0.0);
      backLeft.setPower(0.0);
      backRight.setPower(0.0);
      armMotor.setPower(0.0);
      intakeMotor.setPower(0.0);
   }
}

//code for the arm

