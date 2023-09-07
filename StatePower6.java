
package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import static java.lang.Math.*;
//import java.security.PublicKey; rework controls to become incredibly automated and speed with distance sensors.
// also changed controls to be faster dri
//// Main Code from NOW ON!
//
//// I dont know what to name it so ima just use different shades of meaning for better versions I guess.
//
//// Basically Teleop but completely revamp andver controls
// drive train motor 435rmp ,TICKS_PER_REV = 384.5  WHEEL_RADIUS = 1.88976 1 thicks length =0.7837653 mm;


@TeleOp(name = "StatePower6", group = "A")
@Disabled

public class StatePower6 extends LinearOpMode {

    boolean prorobot = true;
    // boolean prorobot = false;
    Pose2d startPoseA = new Pose2d();

    SampleMecanumDrive drive;
    //drive = new SampleMecanumDrive(hardwareMap);
    private Blinker control_Hub;
    // private DcMotor handle;
    private DcMotor front_left;
    private DcMotorEx front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;
    private DcMotorEx linear_slide;

    private Servo cam;
    private BNO055IMU imu;

    private DigitalChannel take_touch;
    private DigitalChannel guide_touch;
    private DigitalChannel linear_reset;

    private DistanceSensor right_distance;
    private DistanceSensor cam_distance;
    private ColorSensor color;


    private ElapsedTime runtime = new ElapsedTime();
    double encoder = 0;
    int height_stage = 2;
    int linear_slide_current_height = 0;
    boolean has_cone = false;
    boolean manual_adjust = false;
    double start_time = 0;
    double current_time = 0;
    double time_passed = 0;
    boolean touch_guide_flag = false;
    boolean far_dist = false;
    int junction_level = 2;
    boolean bkflag = false;
    boolean take_flag = false;
    boolean intake_error = false;
    double speed_mult = 1.0;
    Pose2d clocation;



    BNO055IMU.Parameters imuParameters; //added
    Orientation angles; //added
    Acceleration gravity; //added

    //Default value is for pro robot;

    int[] outtake_heights = {-1350, -2170, -2990};//preset and down, up will use fixed gap for each level
    //int low[] = {-1340,-1350};  // preset height,outtake height
    //nt mid[] = {-2000,-2204};
    // int high[] = {-2600,-3040};
    int preset_gap = 740; //need check to make sure the cam_sensor catch the pole
    int outtake_down_gap = 85; // before exiting auto control lower linear slide this
    int outtake_up_gap = -250; // after releasing cone raise slide up this gap
    int[] intake_heights = {-10, -390, -505, -620, -735, -850};     // 1 height to 5 height,need check,0 for bn, check;
    int intake_gap = -140; // after intake gap to raise linear slide
    int[] slidespeed = {0, 1900, 2300, 2600};
    int bkheight = 90;
    double tick_length = 0.7837653; // mm

    //Npro robot value
    int[] nouttake_heights = {-1910, -3070, -4230};
    //int nlow[] = {-1500,-1860};  // 0 preset height,1 outtake height
    //int nmid[] = {-2650,-3050};
    //  int nhigh[] = {-3700,-4200};
    int npreset_gap = 1070;
    int nouttake_down_gap = 80; // before exiting auto control lower linear slide this
    int nouttake_up_gap = -260; // after releasing cone raise slide up this gap
    int[] nintake_heights = {0, -510, -668, -826, -985, -1143};     // 1 height to 5 height,0 for bn,check
    int nintake_gap = -180; // after intake gap to raise linear slide, neede check
    int[] nslidespped = {0, 1700, 2300, 2700};
    int nbkheight = 105;
    double ntick_length = 0.584265;

    public void stop_motor() {
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
    }

    public void forward(double power) {
        front_left.setPower(power); //        front_left.setPower(-power);
        front_right.setPower(power);
        rear_left.setPower(power);  //        rear_left.setPower(-power);
        rear_right.setPower(power);
    }

    public void backward(double power) {
        front_left.setPower(-power);//      front_left.setPower(power);
        front_right.setPower(-power);
        rear_left.setPower(-power);  //       rear_left.setPower(power);
        rear_right.setPower(-power);
    }

    public void stop_and_reset() {
        stop_motor();
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder = front_right.getCurrentPosition();
    }


    // returns the  linear slide height in ticks given the cone stack number
    public int stage_to_position(int level) {

        return (intake_heights[level]);
    }

    public void driveturn(int counter, double power) {
        front_left.setPower(power * counter);// front_left.setPower(-power * counter);
        front_right.setPower(-power * counter);
        rear_left.setPower(power * counter);//rear_left.setPower(-power * counter);
        rear_right.setPower(-power * counter);
    }


    private double abs(double input) {
        if (input > 0) return (input);
        else
            return (-input);
    }

    private void linearslidets(int target, int speedlevel) {
        linear_slide.setTargetPosition(target);
        linear_slide.setVelocity(slidespeed[speedlevel]);

    }

    public void spin2(int counter) {
        stop_motor();
        driveturn(counter, 0.7);
        double angle = 0;
        boolean capture_mode = false;
        double target_angle = 0;
        double dist = 0;
        double dist1 = 0;
        double target_distance = 0;
        double time_passed = 0;
        double angle_gap = 2;
        double target_angle1 = 0;
        double target_distance1 = 500;
        Pose2d poseEstimate;


        start_time = runtime.seconds();
        //fast scan to find the junction
        while (time_passed < 0.7 && !capture_mode && opModeIsActive()) {

            if (time_passed > 0.2) driveturn(counter, 0.3);
            dist = cam_distance.getDistance(DistanceUnit.MM);
            // if(dist<60) dist=cam_distance.getDistance(DistanceUnit.MM);
            while ((dist > 700 || dist < 60) && opModeIsActive()&& time_passed<0.6) {
                dist = cam_distance.getDistance(DistanceUnit.MM);
                time_passed = runtime.seconds() - start_time;
            }
            telemetry.addData("fast scan ", dist);
            if (dist < 450) {
                drive.update();
                poseEstimate = drive.getPoseEstimate();
                target_angle = poseEstimate.getHeading();
                // target_angle= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+1*counter;
                target_distance = dist;
                telemetry.addData("Fist",target_distance);
                telemetry.addData("Fist",target_angle);
                capture_mode = true;
                driveturn(-counter, 0.3);
            }
            time_passed = runtime.seconds() - start_time;

        }

        if (time_passed > 0.7) {
            stop_motor();
            return;

        }
        start_time = runtime.seconds();
        time_passed = 0;
        while (time_passed < 0.4 && capture_mode && opModeIsActive()) {
            //driveturn(-counter, 0.2);

            dist = cam_distance.getDistance(DistanceUnit.MM);
            if (dist < 60) dist = cam_distance.getDistance(DistanceUnit.MM);


            telemetry.addData("slow scan ", dist);
            if (dist > dist1 && dist1 < 450 && time_passed > 0.2) capture_mode = false;
            if (dist < target_distance) {
                drive.update();
                poseEstimate = drive.getPoseEstimate();
                angle = poseEstimate.getHeading();
                target_angle1 = target_angle;
                target_angle = angle;
                target_distance1 = target_distance;
                target_distance = dist;

                telemetry.addData("targt angle", angle);

            } else if (dist < target_distance1) {

                drive.update();
                poseEstimate = drive.getPoseEstimate();
                angle = poseEstimate.getHeading();
                target_angle1 = angle;
                target_distance1 = dist;
                //telemetry.addData("slow target1", dist);
            }
            dist1 = dist;
            time_passed = runtime.seconds() - start_time;
        }
        driveturn(counter, 0.3);
       // telemetry.addData("scan target ", target_distance);
        start_time = runtime.seconds();
        while (abs(angle_gap) > 0.02 && time_passed < 0.4 && opModeIsActive()) {
            // dist=cam_distance.getDistance(DistanceUnit.MM);
            dist = cam_distance.getDistance(DistanceUnit.MM);
            if (dist < 60 || dist > 450) dist = cam_distance.getDistance(DistanceUnit.MM);
            telemetry.addData("back ", dist);
            drive.update();
            poseEstimate = drive.getPoseEstimate();
            angle = poseEstimate.getHeading();
            telemetry.addData("back scan ", dist);
            if (dist < target_distance) {
                drive.update();
                poseEstimate = drive.getPoseEstimate();
                angle = poseEstimate.getHeading();
                target_angle1 = target_angle;
                target_angle = angle;
                target_distance1 = target_distance;
                target_distance = dist;

                telemetry.addData("back target angle", angle);

            } else if (dist < target_distance1) {

                drive.update();
                poseEstimate = drive.getPoseEstimate();
                angle = poseEstimate.getHeading();
                target_angle1 = angle;
                target_distance1 = dist;
              //  telemetry.addData("back target1", dist);
            }

            angle_gap = angle - target_angle;
            if (angle_gap > 0) counter = 1;
            else
                counter = -1;
            driveturn(counter, 0.18);

            time_passed = runtime.seconds() - start_time;
        }
        telemetry.addData("back out distance ", dist);
        stop_motor();

        dist = cam_distance.getDistance(DistanceUnit.MM);
        start_time = runtime.seconds();
        if (dist > 450) {
            telemetry.addData("distance measurement over ", dist);
            target_angle = target_angle1;
            angle_gap = 2.0;
            while (abs(angle_gap) > 0.02 && (dist > target_distance) && time_passed < 0.4 && opModeIsActive()) {
                drive.update();
                poseEstimate = drive.getPoseEstimate();
                angle = poseEstimate.getHeading();
                angle_gap = angle - target_angle;
                if (angle_gap > 0) counter = 1;
                else counter = -1;
                driveturn(counter, 0.18);
                dist = cam_distance.getDistance(DistanceUnit.MM);
                time_passed = runtime.seconds() - start_time;

            }
            stop_motor();
            dist = cam_distance.getDistance(DistanceUnit.MM);

        }
        stop_and_reset();
        if (dist > 450) {
            telemetry.addData("distance >450 ", dist);
            telemetry.update();
            return;
        }

        telemetry.addData("final final ", dist);
        telemetry.addData("final fianl angle ", angle);
        // telemetry.addData("Angle measurement",imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES).firstAngle);
        telemetry.update();
        distance_outtake2(dist);

    }



    private void moveforward(double DISTANCE) {

  //  SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d mlocation;
      //clocation = drive.getPoseEstimate();
        drive.update();
        mlocation = drive.getPoseEstimate();
    Trajectory move = drive.trajectoryBuilder(mlocation)
            .forward(DISTANCE)
            .build();
    drive.followTrajectory(move);
    }

    private void location () {
        drive.update();
        Pose2d loc = drive.getPoseEstimate();
        telemetry.addData("finalX", loc.getX());
        telemetry.addData("finalY", loc.getY());
        telemetry.addData("finalHeading",  Math.toDegrees(loc.getHeading()));
        telemetry.update();

    }

    public void spin_dc(int counter) {

        boolean target_flag = false;
        int count=0;

        double dista[] = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,};
        //   double angle[] = {0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        //  double angle_gap;

        int i = 0;
        start_time = runtime.seconds();
        time_passed = 0;
        driveturn(counter, 0.7);
        while ((current_time - start_time)< 0.6 && opModeIsActive()) {

            dista[i] = cam_distance.getDistance(DistanceUnit.MM);
            current_time = runtime.seconds();
            i++;
        }
        stop_motor();
        count=i;
        telemetry.addData("count ", count);

        for (int j = 0; j < count; j++) {
            telemetry.addData("distance measurement ", dista[j]);
        }
        telemetry.update();
        while ( opModeIsActive()&& !gamepad1.x)
        {

        }

    }



    private void fastscore() {

        //  SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d mlocation;
        Pose2d c2location;
        //clocation = drive.getPoseEstimate();
        linearslidets(outtake_heights[3]+preset_gap,2000);
        drive.update();
        clocation = drive.getPoseEstimate();
        mlocation=  new Pose2d(clocation.getX(),clocation.getY()-50, (clocation.getHeading())+Math.toRadians(40));
        Trajectory fastf = drive.trajectoryBuilder(clocation)
                .lineToSplineHeading(mlocation)
                .build();
        drive.followTrajectory(fastf);
        spina(-1);
        sleep(200);
        stop_motor();
        drive.update();
        c2location = drive.getPoseEstimate();
        Trajectory fastb = drive.trajectoryBuilder(c2location)
                .lineToSplineHeading(clocation)
                .build();
        drive.followTrajectory(fastb);
    }
    public void spina(int counter) {
        stop_motor();
        driveturn(counter, 0.3);

        double angle;
        boolean back_mode=false;
        double target_angle = 0;
        double dist;
        double target_distance = 450;
        double time_passed=0;
        double angle_gap=2 ;
        double target_angle1=0;
        double target_distance1=500;
        Pose2d poseEstimate;
        int  count=0;

        start_time = runtime.seconds();
        while (time_passed< 0.5 && !back_mode && opModeIsActive()) {

            dist = cam_distance.getDistance(DistanceUnit.MM);
            while ((dist>600 ||dist<60)&& opModeIsActive())
            {
                dist = cam_distance.getDistance(DistanceUnit.MM);
            }

            if (dist<target_distance) {
                target_angle1= target_angle;
                drive.update();
                poseEstimate=drive.getPoseEstimate();
                target_angle=poseEstimate.getHeading();
                target_distance1=target_distance;
                target_distance=dist;
                count++;
                telemetry.addData("target",dist);
            }
            else if (dist<target_distance1) {
                drive.update();
                poseEstimate=drive.getPoseEstimate();
                target_angle1=poseEstimate.getHeading();
                target_distance1=dist;
                telemetry.addData("t Angle1",target_angle1);
            }

            time_passed=runtime.seconds()-start_time;

            if (count>0) {
                back_mode = true;
                driveturn(-counter, 0.25);
            }
            // telemetry.addData("fdist",dist);
        }

        start_time = runtime.seconds();
        time_passed=0;

        while (abs(angle_gap) > 0.02 && opModeIsActive()&&time_passed<0.4) {
            dist=cam_distance.getDistance(DistanceUnit.MM);
            drive.update();
            poseEstimate=drive.getPoseEstimate();
            angle=poseEstimate.getHeading();
            telemetry.addData("back scan ",dist);
            telemetry.addData("back Angle",angle);
            if (dist<target_distance) {
                target_angle1= target_angle;
                target_angle=angle;
                target_distance1=target_distance;
                target_distance=dist;
                //telemetry.addData("target Angle",target_angle);
            }
            else if (dist<target_distance1) {
                target_angle1= angle;
                target_distance1=dist;
                //telemetry.addData("target1 Angle",target_angle1);
            }
            angle_gap = angle - target_angle;
            if (angle_gap > 0) counter = 1;
            else
                counter = -1;
            driveturn(counter, 0.25 );
            time_passed=runtime.seconds()-start_time;
        }
        stop_motor();

        dist=cam_distance.getDistance(DistanceUnit.MM);
        telemetry.addData("final",dist);

        if(dist>350) {
            telemetry.addData("distance measurement over ",dist);
            target_angle=target_angle1;
            angle_gap=2.0;
            while (abs(angle_gap) > 0.02 && opModeIsActive()) {
                drive.update();
                poseEstimate=drive.getPoseEstimate();
                angle_gap = poseEstimate.getHeading()- target_angle;
                if (angle_gap > 0) counter = 1;
                else counter = -1;
                driveturn(counter, 0.25 );
            }

            dist=cam_distance.getDistance(DistanceUnit.MM);
        }
        stop_motor();
        // front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //     front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //   drive.update();
        //  poseEstimate=drive.getPoseEstimate();
        // telemetry.addData("distance measurement ",dist);
        // telemetry.addData("Angle measurement",  poseEstimate.getHeading());
        //   telemetry.update();
        // sleep(5000000);
        //if(dist>350) return;//ACCIDENT PRCCESSING
        distance_outtake2(dist);

    }
    public void distance_outtake22(double spin_distance) {

        int  bkadjusment=0;


        if(bkflag) bkadjusment=bkheight;
        if (!manual_adjust) {
            stop_and_reset();
            double drive_distance=0;
            boolean far=false;
            int start_encoder;
            double distance_current;

            touch_guide_flag = false;
            far_dist = false;
            start_time = runtime.seconds();
            time_passed = 0;// old 0.3
            if(spin_distance>0) spin_distance= cam_distance.getDistance(DistanceUnit.MM);
            linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
            while (opModeIsActive() && linear_slide.getCurrentPosition() > (outtake_heights[junction_level] + preset_gap-300 ) && time_passed < 1.0) {// +10 for tolerance
                current_time = runtime.seconds();
                time_passed = current_time - start_time;

                if (gamepad2.a) {
                    junction_level = 0;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (gamepad2.b) {
                    junction_level = 1;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (gamepad2.y) {
                    junction_level = 2;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }

            }


            distance_current=right_distance.getDistance(DistanceUnit.MM) ;

            if(spin_distance>0) drive_distance=spin_distance/tick_length-24;
            else if (distance_current>300) far=true;
            else drive_distance= distance_current/2.0-23;//2.4 is for a ratio

            //   telemetry.addData("orgial distance",distance_current);


            start_encoder= front_right.getCurrentPosition();
            telemetry.addData("start encoder",start_encoder);
            encoder=start_encoder;
            forward(0.35);
            start_time = runtime.seconds();
            time_passed = 0;

            while ( right_distance.getDistance(DistanceUnit.MM) > 210 && !touch_guide_flag &&time_passed < 1.0&& far && opModeIsActive() )
            {

                time_passed = runtime.seconds() - start_time;
                if (gamepad2.a) {
                    junction_level = 0;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (gamepad2.b) {
                    junction_level = 1;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (gamepad2.y) {
                    junction_level = 2;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }

            }

            // encoder for short distance and spin distance.

            while (opModeIsActive() &&(encoder-start_encoder)< drive_distance && !touch_guide_flag &&time_passed < 1.0&& !far)
            {

                time_passed = runtime.seconds()- start_time;
                encoder= front_right.getCurrentPosition();
                distance_current=right_distance.getDistance(DistanceUnit.MM);
                if (distance_current <170) {
                    telemetry.addData("distance break",distance_current);
                    break;

                }



                if (gamepad2.a) {
                    junction_level = 0;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (gamepad2.b) {
                    junction_level = 1;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (gamepad2.y) {
                    junction_level = 2;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }


            }

            if (!guide_touch.getState()) touch_guide_flag = true;
            if(far) sleep(40);
            telemetry.addData("end encoder",encoder);

            if (!guide_touch.getState()) touch_guide_flag = true;
            if ( touch_guide_flag) {

                backward(0.2);
                sleep(150);
                // telemetry.addData("press and back,touch flg2",touch_guide_flag);
                touch_guide_flag=false;
                //  telemetry.update();

            }

            stop_motor();
            while (opModeIsActive() && linear_slide.getCurrentPosition() - (outtake_heights[junction_level]+bkadjusment) > 10) {

                if (gamepad2.a) {
                    junction_level = 0;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (gamepad2.b) {
                    junction_level = 1;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (gamepad2.y) {
                    junction_level = 2;
                    linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
                }
                if (!guide_touch.getState()) touch_guide_flag = true;
            }


            if ( touch_guide_flag) {

                backward(0.2);
                sleep(150);
                // telemetry.addData("press and back,touch flg2",touch_guide_flag);
                touch_guide_flag=false;
                //  telemetry.update();
                stop_motor();
            }


            //stop_motor();
            manual_adjust = true;
            return;
        }


        // stop_motor();
        cam.setPosition(0);
        sleep(350);
        if (bkflag) sleep(100);
        bkflag=false;
        manual_adjust = false;
        linearslidets(outtake_heights[junction_level] + bkadjusment+outtake_up_gap, 3);
        sleep(200);
        backward(0.5);
        sleep(200);
        stop_motor();

        // if (height_stage > 1) height_stage--;
        linearslidets(intake_heights[height_stage], 2);
        junction_level = 2;//default set to H
        // telemetry.addData("outtake2",color.argb());
        //  telemetry.update();
        has_cone = false;
        // telemetry.addData("guide_touch, false pressed",guide_touch.getState());
        //   telemetry.update();
    }

    public void spin2a(int counter) {
        stop_motor();
        driveturn(counter, 0.7);
        double angle=0;
        boolean capture_mode=false;
        double target_angle = 0;
        double center_angle=0;
        double dist=0;
        double dist1=0;
        double target_distance = 0;
        double time_passed=0;
        double angle_gap=2 ;
        double target_angle1=0;
        double target_distance1=500;
        boolean slow_mode=false;
        double range[]={0.2,0.1,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02};
        double drivepower=0.7;
        int turn=0;
        int count=0;
        Pose2d poseEstimate;
        drive.update();
        poseEstimate=drive.getPoseEstimate();

        start_time = runtime.seconds();
        //fast scan to find the junction
        while (time_passed< 0.7 && !capture_mode && opModeIsActive()) {

            if(time_passed>0.2) drivepower=0.2;
            driveturn(counter, drivepower);
            dist = cam_distance.getDistance(DistanceUnit.MM);
            // if(dist<60) dist=cam_distance.getDistance(DistanceUnit.MM);
            while ((dist>500|| dist<60) && opModeIsActive())
            {
                dist = cam_distance.getDistance(DistanceUnit.MM);
            }
            telemetry.addData("fast scan ",dist);
            if (dist<400) {
                drive.update();
                poseEstimate=drive.getPoseEstimate();
                target_angle=poseEstimate.getHeading()  ;
                target_distance=dist;
                telemetry.addData("target angle",target_angle);
                capture_mode=true;
                counter=-counter;
                drivepower=0.4;
                driveturn(counter, drivepower);
            }
            time_passed=runtime.seconds()-start_time;

        }

        if(time_passed>0.7) {
            stop_motor();
            return;
        }

        start_time = runtime.seconds();
        time_passed=0;
        while (time_passed< 0.5 &&(count<3 || angle_gap>range[count]) && opModeIsActive()) {
            //driveturn(-counter, 0.2);
            dist = cam_distance.getDistance(DistanceUnit.MM);
            while(dist<60 || dist>500) {
                dist = cam_distance.getDistance(DistanceUnit.MM);
            }
            if((time_passed>0.2||dist<400)&&!slow_mode)  {
                driveturn(counter, 0.3);
                slow_mode=true;
            }
            drive.update();
            poseEstimate=drive.getPoseEstimate();
            angle=poseEstimate.getHeading();
            telemetry.addData("angel",angle);
            if (dist<target_distance) {

                target_angle1= target_angle;
                target_angle=angle;
                target_distance1=target_distance;
                target_distance=dist;
                count++;
                drivepower=0.15;

                telemetry.addData("slow target",dist);

            }
            else if (dist<target_distance1) {

                target_angle1= angle;
                target_distance1=dist;
               //  telemetry.addData("slow target1",dist);
            }

                angle_gap = angle- target_angle;
                if (angle-target_angle-range[count] > 0) counter = 1;
                else if(angle-target_angle+range[count] <0) counter = -1;
            telemetry.addData("counter ",counter);
            driveturn(counter, drivepower);
              telemetry.addData("slow scan ",dist);


            time_passed=runtime.seconds()-start_time;
        }


        telemetry.addData("back out distance ",dist);
        stop_motor();

        dist=cam_distance.getDistance(DistanceUnit.MM);
        start_time=runtime.seconds();
        if(dist>300) {
            telemetry.addData("distance measurement over ",dist);
            target_angle=target_angle1;
            angle_gap=2.0;
            while (abs(angle_gap) > 0.026 && (dist>target_distance)&&time_passed<0.3 && opModeIsActive())  {
                drive.update();
                poseEstimate=drive.getPoseEstimate();
                angle=poseEstimate.getHeading();
                angle_gap = angle- target_angle;
                if (angle_gap > 0) counter = 1;
                else counter = -1;
                driveturn(counter, drivepower );
                dist=cam_distance.getDistance(DistanceUnit.MM);
                time_passed=runtime.seconds()-start_time;

            }
            stop_motor();
            dist=cam_distance.getDistance(DistanceUnit.MM);

        }
        telemetry.addData("final final ",dist);
        telemetry.addData("final fianl angle ",angle);
        telemetry.update();

        stop_and_reset();
        if(dist>300)        {
            telemetry.addData("distance >300 ",dist);
            telemetry.update();
            return;
        }


        // telemetry.addData("Angle measurement",imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY,AngleUnit.DEGREES).firstAngle);
        telemetry.update();
      //  distance_outtake2(dist);
      //  sleep (2000);
    }

    public void distance_outtake2(double spin_distance) {
        sleep(100);
        int  bkadjusment=0;
        double y,x,rx,denominator,frontLeftPower,backLeftPower,frontRightPower,backRightPower;
        if(bkflag) bkadjusment=bkheight;
        boolean far=false;
        touch_guide_flag = false;
        spin_distance= cam_distance.getDistance(DistanceUnit.INCH)+1.58;
        if(spin_distance>18||spin_distance<2) spin_distance=0;
        linearslidets(outtake_heights[junction_level]+bkadjusment, 3);
        start_time = runtime.seconds();
        time_passed = 0;
        while (opModeIsActive() && linear_slide.getCurrentPosition() > (outtake_heights[junction_level] + 600 ) && time_passed < 1.0) {
                time_passed = runtime.seconds() - start_time;
                dropheight(bkadjusment);
            }

            if(spin_distance>0) moveforward(spin_distance);
            else  far=true;

             if(far)  {
                 forward(0.35);
                 sleep(150);
             }
            start_time = runtime.seconds();
            time_passed = 0;

            while ( right_distance.getDistance(DistanceUnit.MM) > 210 && !touch_guide_flag &&time_passed < 1.0&& far && opModeIsActive() )
            {
                time_passed = runtime.seconds() - start_time;
                dropheight(bkadjusment);

            }
            if (far) sleep(40);
;

            stop_motor();


            if ( !guide_touch.getState()) {

                backward(0.2);
                sleep(150);
               touch_guide_flag=false;
               stop_motor();
            }
        start_time = runtime.seconds();
        time_passed = 0;
//        while (opModeIsActive() && abs(linear_slide.getCurrentPosition() - (outtake_heights[junction_level]+bkadjusment)) > 20) {
//            dropheight(bkadjusment);
//            time_passed=    runtime.seconds()-start_time;
//        }
        while (opModeIsActive() && !gamepad1.x&&!gamepad1.y||(linear_slide.getCurrentPosition() - (outtake_heights[junction_level]+bkadjusment)) > 10) {
                y = gamepad1.right_stick_y; // Remember, this is reversed!
                x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
                rx = -gamepad1.left_stick_x * 0.75;
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;
                front_left.setPower(-frontLeftPower*0.3);
                front_right.setPower(-frontRightPower*0.3);
                rear_left.setPower(-backLeftPower*0.3);
                rear_right.setPower(-backRightPower*0.3);
                dropheight(bkadjusment);
             }
        stop_motor();
        cam.setPosition(0);
        sleep(300);
        bkflag=false;
        manual_adjust = false;
        linearslidets(outtake_heights[junction_level] + bkadjusment+outtake_up_gap, 3);
        sleep(150);
        backward(0.6);
        sleep(175);
        stop_motor();
        //height_stage=2;
        linearslidets(intake_heights[height_stage], 2);
        junction_level = 2;//default set to H
        has_cone = false;
        start_time = runtime.seconds();

    }
    private void pickCone(){
        // TODO test for the right heights and enough wait time
        forward(0.18);
        linearslidets(intake_heights[5],3);

        boolean guide_touch_flag=false;
       boolean  take_touch_flag=false;

        sleep(150);
        runtime.reset();
        while (!guide_touch_flag && opModeIsActive() && runtime.seconds() <1.2){
            if(!guide_touch.getState()&&abs(front_right.getVelocity())<5) guide_touch_flag=true;
        }
        stop_motor();
        linearslidets(intake_heights[0],1);

        while(opModeIsActive() && !take_touch_flag && runtime.seconds() < 1.2){

            take_touch_flag=!take_touch.getState()&&(linear_slide.getCurrentPosition()>-700);
        }

        cam.setPosition(0.33);
        // drive.setMotorPowers(0,0,0,0);
        linear_slide.setVelocity(0);
        has_cone=true;

        sleep(300);
        //current_pos = linear_slide.getCurrentPosition();
        linearslidets( -1200, 2);

        while (linear_slide.getCurrentPosition() > -900 ){
            // slidePositionTo(linear_slide.getCurrentPosition() - 500, 2800, 0);
        }
        backward(0.4);
        sleep(200);
    }



    public void linear_sensor_reset() {

        stop_motor();
        cam.setPosition(0);
        has_cone = false;
        manual_adjust=false;
        bkflag=false;
        sleep(350);
        sleep(150);
        backward(0.6);
        sleep(100);
        stop_motor();
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        time_passed=0;
        start_time=runtime.seconds();
        linear_slide.setPower(0.6);
        while (opModeIsActive() && linear_reset.getState()&&(time_passed<1.5)) {

            time_passed=runtime.seconds()-start_time;

        }
        linear_slide.setPower(0);
        sleep(100);
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setTargetPosition(intake_heights[5]);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        height_stage = 2;
        linearslidets(intake_heights[height_stage], 3);
    }
    private void dropheight(int bk){
        if (gamepad2.a) {
            junction_level = 0;
            linearslidets(outtake_heights[junction_level]+bk, 3);
        }
        if (gamepad2.b) {
            junction_level = 1;
            linearslidets(outtake_heights[junction_level]+bk, 3);
        }
        if (gamepad2.y) {
            junction_level = 2;
            linearslidets(outtake_heights[junction_level]+bk, 3);
        }

    }




    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        // handle = hardwareMap.get(DcMotor.class, "handle");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        linear_slide = hardwareMap.get(DcMotorEx.class, "linear_slide");
        cam = hardwareMap.get(Servo.class, "cam");
        take_touch = hardwareMap.get(DigitalChannel.class, "take_touch");
        guide_touch = hardwareMap.get(DigitalChannel.class, "guide_touch");
        linear_reset = hardwareMap.get(DigitalChannel.class, "linear_reset");
        right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");
        cam_distance = hardwareMap.get(DistanceSensor.class, "cam_distance");
        color = hardwareMap.get(ColorSensor.class,"color");
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //added
        // imu = hardwareMap.get(Gyroscope.class, "imu");


        double y,x,rx,denominator,frontLeftPower,backLeftPower,frontRightPower,backRightPower;
     //   double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
     //   double rx = -gamepad1.left_stick_x * 0.75;
     //   double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
     //   double frontLeftPower = (y + x + rx) / denominator;
      //  double backLeftPower = (y - x + rx) / denominator;
      //  double frontRightPower = (y - x - rx) / denominator;
      //  double backRightPower = (y + x - rx) / denominator;

        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //  imuParameters.calibrationDataFile=
        // Disable logging.
        imuParameters.loggingEnabled = false;
        cam_distance.resetDeviceConfigurationForOpMode();
        right_distance.resetDeviceConfigurationForOpMode();
        take_touch.setMode(DigitalChannel.Mode.INPUT);
        guide_touch.setMode(DigitalChannel.Mode.INPUT);
        linear_reset.setMode(DigitalChannel.Mode.INPUT);
        imu.initialize(imuParameters);
        color.enableLed(false);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!prorobot) // set all the value to nonpro robot
        {
            outtake_heights[0] = nouttake_heights[0];
            outtake_heights[1] = nouttake_heights[1];
            outtake_heights[2] = nouttake_heights[2];
            preset_gap = npreset_gap;
            outtake_down_gap = nouttake_down_gap;
            outtake_up_gap = nouttake_up_gap;
            intake_heights[0] = nintake_heights[0];
            intake_heights[1] = nintake_heights[1];
            intake_heights[2] = nintake_heights[2];
            intake_heights[3] = nintake_heights[3];
            intake_heights[4] = nintake_heights[4];
            intake_heights[5] = nintake_heights[5];
            intake_gap = nintake_gap;
            slidespeed[0] = nslidespped[0];
            slidespeed[1] = nslidespped[1];
            slidespeed[2] = nslidespped[2];
            bkheight=nbkheight;
            tick_length=ntick_length;

        }
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPoseA);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setTargetPosition(intake_heights[5]);
        waitForStart();
        runtime.reset();
        cam.setPosition(0);

        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslidets(intake_heights[5], 3);
        stop_and_reset();
        drive.update();

        while (opModeIsActive()) {

            // Driver controls from Game Manual 0

            height_stage = 2;
//            speed_mult = 1.0;
             y = gamepad1.right_stick_y; // Remember, this is reversed!
             x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
             rx = -gamepad1.left_stick_x * 0.75;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
             frontLeftPower = (y + x + rx) / denominator;
             backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
             backRightPower = (y + x - rx) / denominator;

            //front_left.setPower(frontLeftPower*speed_mult);
            front_left.setPower(-frontLeftPower*speed_mult);
            front_right.setPower(-frontRightPower*speed_mult);
//            rear_left.setPower(backLeftPower*speed_mult);
            rear_left.setPower(-backLeftPower*speed_mult);
            rear_right.setPower(-backRightPower*speed_mult);

            if (gamepad1.left_trigger > 0.3){
                speed_mult = 0.75;
                telemetry.addData("speed change",speed_mult);
                telemetry.update();
            }
            if (gamepad1.right_trigger > 0.3) {
                speed_mult = 1.0;
            }


            if ( (gamepad1.y || gamepad1.x) && has_cone){
                distance_outtake2(0);
            }

//            telemetry.addData("Current Cam Distance", cam_distance.getDistance(DistanceUnit.MM));
//            // telemetry.addData("Velocity", front_right.getVelocity());
//            //   telemetry.addData("Current Right Distance",right_distance.getDistance(DistanceUnit.MM));
//            telemetry.update();


            if (gamepad2.a && has_cone) {
                junction_level = 0;
                if(manual_adjust)
                    linearslidets(outtake_heights[junction_level], 3);
                else
                    linearslidets(outtake_heights[junction_level] + preset_gap, 3);
            }

            if (gamepad2.b && has_cone) {
                junction_level = 1;
                if(manual_adjust)
                    linearslidets(outtake_heights[junction_level], 3);
                else
                    linearslidets(outtake_heights[junction_level] + preset_gap, 3);
            }

            if (gamepad2.y && has_cone) {
                junction_level = 2;
                if(manual_adjust)
                    linearslidets(outtake_heights[junction_level], 3);
                else
                    linearslidets(outtake_heights[junction_level] + preset_gap, 3);
            }

            if (gamepad2.x) {
                stop_motor();
                cam.setPosition(0);
                has_cone = false;
                manual_adjust=false;
                bkflag=false;
                sleep(350);
                linearslidets(linear_slide.getCurrentPosition() + outtake_up_gap, 3);
                sleep(150);
                backward(0.6);
                sleep(100);
                stop_motor();
                // sleep(100);
                if (height_stage > 1) height_stage--;
                linearslidets(intake_heights[height_stage], 2);
            }

            // linear slide goes down slowly until touch sensor is pressed check? right?
           // if ((gamepad2.dpad_right && !has_cone) || (!has_cone && !guide_touch.getState()))

            if ((gamepad2.dpad_right || !guide_touch.getState()) && !has_cone && runtime.seconds() - start_time > 1)
                {
                stop_motor();
                linearslidets(intake_heights[0], 2);
                while (opModeIsActive() && !intake_error && !take_flag) {
                    if (gamepad1.b) intake_error = true;
                    take_flag = !take_touch.getState() && (linear_slide.getCurrentPosition()>-700);
                    if(linear_slide.getCurrentPosition()>intake_heights[5])
                        linearslidets(intake_heights[0], 1);
                  //  time_passed=runtime.seconds()-start_time;
                }
                linear_slide.setVelocity(0);
                if (!intake_error) {
                    cam.setPosition(0.33);
                    has_cone = true;
                    sleep(450);
                }
                take_flag = false;
                if (linear_slide.getCurrentPosition()>-80 && runtime.seconds() >85&&!intake_error ) bkflag=true;
                linearslidets(outtake_heights[1] + preset_gap+200, 3);//check why not intake height;


                linear_slide_current_height = linear_slide.getCurrentPosition();
               // while (opModeIsActive() && linear_slide_current_height > (linear_slide.getTargetPosition() + 2))//add 2 as tolerance;
                    while (opModeIsActive() && linear_slide_current_height > -250)//add 2 as tolerance;
                {
                    if (gamepad1.b) intake_error = true;
                    linear_slide_current_height = linear_slide.getCurrentPosition();
                }
                if (!intake_error) stop_motor();
                else if (intake_error){
                    backward(0.5);
                    sleep(200);
                    stop_motor();
                }
                intake_error = false;
                start_time = runtime.seconds();


            }
            //  if (gamepad2.right_bumper&& has_cone) bkflag=true; use intake height to set the bkflag;

            //if(gamepad1.a) moveforward(4);// only for test
            if (gamepad2.dpad_up) {
                //wait_until(!gamepad2.dpad_up);
                height_stage = 5;
                linearslidets(intake_heights[height_stage], 3);
            }
            if (gamepad2.dpad_down) {
                //wait_until(!gamepad2.dpad_down);
                height_stage = 1;
                linearslidets(intake_heights[height_stage], 3);
            }

            if (gamepad2.right_bumper && gamepad2.dpad_left) {  //&& gamepad2.right_bumper
                linear_sensor_reset();
            }


            if (gamepad1.left_bumper&&has_cone) {
                spin2(-1);
            }
            if (gamepad1.right_bumper&&has_cone) {
                spin2(1);
            }
            if(gamepad2.left_bumper) pickCone();
           // telemetry.addData("Right Distance",right_distance.getDistance(DistanceUnit.MM));
          //  telemetry.addData("cam distance", cam_distance.getDistance(DistanceUnit.MM));
            // color.enableLed(true);
          //  telemetry.addData("Alpha integer",color.alpha());
          // telemetry.addData("Red integer",color.red());
          //  telemetry.addData("Green integer",color.green());
           // telemetry.addData("Blue integer",color.blue());
           //  telemetry.addData("ARGB Value",color.argb());
            //telemetry.update();



        }

    }
}