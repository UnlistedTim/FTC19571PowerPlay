package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp

public class PowerTeleV1 extends LinearOpMode {
    BaseClass rbg;
// intake and driving  -driver gamepad1
//   right_bumper  intake
//   left.bmper  transfer
// right   left_trigger  intake slides



    //outtake --gunner game pad2
    // gamepad 2:right sticky  outtake slides
      // bumper: turn table
    // square : drop
    // dpad left : outtkae-arm
    //drap up : outtake handle;
// left and right trigger  . 100 OUTtake  LINEAR SLIDES
    //triangle --- out arm and handle high (arm value decrease)    x counter direction

    public void runOpMode() throws InterruptedException {

        rbg = new BaseClass(hardwareMap);
        int junction_level=3;
        waitForStart();
        rbg.intake_ready();
        sleep(500);
       rbg.outtake_ready();



        while (opModeIsActive()) {

            if (gamepad1.right_bumper) rbg.intake();
            if (gamepad1.left_bumper) rbg.transfer();

//           if( gamepad2.right_stick_y>0.1 || gamepad2.right_stick_y<-0.1) {
//              rbg.adjust_outtake_slide(-gamepad2.right_stick_y);
//
//               sleep(300);
//               telemetry.addData("Slide Pos",rbg.outtake_right.getCurrentPosition());
//               telemetry.addData("Out Arm pos",rbg.outtake_arm.getPosition());
//               telemetry.addData("Out handle pos",rbg.outtake_handle.getPosition());
//               telemetry.update();
//
//           }
            if(gamepad1.left_trigger>0.2)  rbg.adjust_intake_slide(-gamepad1.left_trigger);
            if(gamepad1.right_trigger>0.2)  rbg.adjust_intake_slide(gamepad1.right_trigger);


            if (gamepad2.right_bumper){
               // rbg.control_turn.setPosition(0);
                rbg.carousel_turn(true);
            }

            if (gamepad2.left_bumper){
               // rbg.expansion_turn.setPosition(0);
               rbg.carousel_turn(false);

            }
            if (gamepad2.square) {
                    rbg.manual_drop();
                    junction_level=3;
                
            }
            if(gamepad2.left_trigger>0.1) {
                rbg.adjust_outtake_slide(-gamepad2.left_trigger,junction_level);

            }
            if(gamepad2.right_trigger>0.1) {
                rbg.adjust_outtake_slide(gamepad2.right_trigger,junction_level);

            }
            if(gamepad2.dpad_left ) rbg.out_arm_adjust(1);
            if (gamepad2.dpad_right) rbg.out_arm_adjust(-1);

            if(gamepad2.dpad_up ) rbg.out_handle_adjust(1);
            if (gamepad2.dpad_down) rbg.out_handle_adjust(-1);



            if (gamepad2.triangle){

                rbg.out_arm_hundle(-1);

            }
            if (gamepad2.cross){

                rbg.out_arm_hundle(1);

            }


            telemetry.addData("Slide Pos",rbg.outtake_right.getCurrentPosition());
            telemetry.addData("Out Arm pos",rbg.outtake_arm.getPosition());
            telemetry.addData("Out handle pos",rbg.outtake_handle.getPosition());
            telemetry.addData("in Slide Pos",rbg.intake_right.getCurrentPosition());
            telemetry.addData("red",rbg.grab_color.red());
            telemetry.addData("blue",rbg.grab_color.blue());
            telemetry.update();

            rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);


        }
    }
}

