
package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.externseral.navigation.Orientation;
//import static java.lang.Math.*;
//import java.security.PublicKey;// rework controls to become incredibly automated and speed with distance sensors.
// also changed controls to be faster dri
//// Main Code from NOW ON!
//
//// I dont know what to name it so ima just use different shades of meaning for better versions I guess.
//
//// Basically Teleop but completely revamp andver controls
// drive train motor 435rmp ,TICKS_PER_REV = 384.5  WHEEL_RADIUS = 1.88976 1 thicks length =0.7837653 mm;


@TeleOp(name = "Demo", group = "A")

public class Demo extends LinearOpMode {


    //Pose2d startPoseA = new Pose2d();

  SampleMecanumDrive drive;
    //drive = new SampleMecanumDrive(hardwareMap);
    private Blinker control_Hub;
     //private DcMotor handle;
    private DcMotor front_left;
    private DcMotorEx front_right;
    private DcMotor rear_left;
    private DcMotor rear_right;
    private DcMotorEx linear_slide;

    private Servo cam;
    private Servo cam_turn;
    private BNO055IMU imu;

    private DigitalChannel take_touch;
    private DigitalChannel guide_touch;
    private DigitalChannel linear_reset;
    private DistanceSensor cdis;
    private DistanceSensor right_distance;
//    private DistanceSensor cam_distance;
    private DistanceSensor left_distance;
    private ColorSensor color;

    private VoltageSensor batteryVoltageSensor;


    private ElapsedTime runtime = new ElapsedTime();
    double encoder = 0;
    int height_stage = 1;
    boolean has_cone = false;
    double start_time = 0;
    double time_passed = 0;
    boolean touch_guide_flag = false;
    int junction_level = 1;
    boolean take_flag = false;
    boolean intake_error = false;
    double default_speed = 0.4;
    double speed_mult = 0.4;
    boolean slow_speed_flag=false;
    boolean manual = false;
 //   Pose2d clocation;




    BNO055IMU.Parameters imuParameters; //added
  //Orientation angles; //added
  //Acceleration gravity; //added

    //Default value is for pro robot;

    int[] outtake_heights = {-990, -1820,-2210, -2640};//preset and down, up will use fixed gap for each level 1350
   // int preset_gap = 740; //need check to make sure the cam_sensor catch the pole
    int outtake_up_gap = -300; // after releasing cone raise slide up this gap
    int[] intake_heights = {30, -270, -400, -620, -735, -695};     // 1 height to 5 height,need check,0 for bn, check;
  //  int intake_gap = -140; // after intake gap to raise linear slide
    int[] slidespeed = {0, 1800, 2300, 2600};

   // double tick_length = 0.7837653; // mm




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
        if(junction_level==2) {
            junction_level=3;
            linearslidets(outtake_heights[junction_level],3 );
        }
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
        int count=0;
        int count1=0;
        Pose2d poseEstimate;


        start_time = runtime.seconds();
        //fast scan to find the junction
        while (time_passed < 0.7 && !capture_mode && opModeIsActive()) {

            if (time_passed > 0.3) driveturn(counter, 0.4);
            dist = cdis.getDistance(DistanceUnit.MM);
           // telemetry.addData("Fscan",target_distance);
         //   telemetry.addData("Fist",target_angle);
            if (dist < 450&&dist>70) {
                drive.update();
                poseEstimate = drive.getPoseEstimate();
                target_angle = poseEstimate.getHeading();
                target_distance = dist;
                telemetry.addData("Fist",target_distance);
                telemetry.addData("Fist",target_angle);
                capture_mode = true;
                driveturn(-counter, 0.35);
            }
            time_passed = runtime.seconds() - start_time;

        }

        if (time_passed > 0.7) {
            stop_motor();
            return;
        }
        start_time = runtime.seconds();
        time_passed = 0;
        while (time_passed < 0.6 && count1<2 && opModeIsActive()) {
            dist = cdis.getDistance(DistanceUnit.MM);
            count++;
            if (dist < 70) dist = cdis.getDistance(DistanceUnit.MM);
            telemetry.addData("slow scan ", dist);
            if(dist < 450& count>5)    driveturn(-counter, 0.20);
            if (count1>0 && ( dist>dist1||dist>450) ) count1++;
            if (dist>dist1 && dist1 < 450 && count>5) count1++;
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
        driveturn(counter, 0.18);
        // telemetry.addData("scan target ", target_distance);
        time_passed=0;
        start_time = runtime.seconds();
        while (abs(angle_gap) > 0.03 && time_passed < 0.6 && opModeIsActive()) {
            // dist=cdis.getDistance(DistanceUnit.MM);
            dist = cdis.getDistance(DistanceUnit.MM);
            if (dist < 70 || dist > 550) dist = cdis.getDistance(DistanceUnit.MM);
            telemetry.addData("back ", dist);
            drive.update();
            poseEstimate = drive.getPoseEstimate();
            angle = poseEstimate.getHeading();
           // telemetry.addData("back scan ", dist);
            if (dist < target_distance) {
               // drive.update();
              //  poseEstimate = drive.getPoseEstimate();
              //  angle = poseEstimate.getHeading();
                target_angle1 = target_angle;
                target_angle = angle;
                target_distance1 = target_distance;
                target_distance = dist;

                telemetry.addData("back target angle", angle);

            }
            else if (dist < target_distance1) {

               // drive.update();
               // poseEstimate = drive.getPoseEstimate();
              //  angle = poseEstimate.getHeading();
                target_angle1 = angle;
                target_distance1 = dist;
                //  telemetry.addData("back target1", dist);
            }

            angle_gap = angle - target_angle;
            if ((angle_gap > 0&& angle_gap<3.1416)||(angle_gap< 0&& angle_gap<-3.1416)) counter = 1;
            else
                counter = -1;
            driveturn(counter, 0.15);

            time_passed = runtime.seconds() - start_time;
            if (abs(angle_gap)>3.1416)  angle_gap=6.2832-abs(angle_gap);

        }
        stop_motor();
        dist = cdis.getDistance(DistanceUnit.MM);
        telemetry.addData("back out distance ", dist);
        telemetry.addData("back out dngle ", angle);


        start_time = runtime.seconds();
        if (dist > 450) {
            telemetry.addData("distance measurement over ", dist);
            target_angle = target_angle1;
            angle_gap = 2.0;
            while (abs(angle_gap) > 0.03 && (dist > target_distance) && time_passed < 0.4 && opModeIsActive()) {
                drive.update();
                poseEstimate = drive.getPoseEstimate();
                angle = poseEstimate.getHeading();
                angle_gap = angle - target_angle;
                if ((angle_gap > 0&& angle_gap<3.1416)||(angle_gap< 0&& angle_gap<-3.1416)) counter = 1;
                else
                    counter = -1;
                driveturn(counter, 0.15);

                time_passed = runtime.seconds() - start_time;
                if (abs(angle_gap)>3.1416)  angle_gap=6.2832-abs(angle_gap);
            }
         stop_motor();
        }

        dist = cdis.getDistance(DistanceUnit.MM);
        if (dist > 450) {
            telemetry.addData("distance >450 ", dist);
            telemetry.update();
            return;
        }

        telemetry.addData("final final ", dist);
        telemetry.addData("final fianl angle ", angle);
        telemetry.update();
      distance_outtake2();

    }



//    private void moveforward(double DISTANCE) {
//
//        //  SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Pose2d mlocation;
//        //clocation = drive.getPoseEstimate();
//        drive.update();
//        mlocation = drive.getPoseEstimate();
//        Trajectory move = drive.trajectoryBuilder(mlocation)
//                .forward(DISTANCE)
//                .build();
//        drive.followTrajectory(move);
//    }




    public void distance_outtake2() {
        double distance;
        //double y1,x1,rx1,denominator1,frontLeftPower1,backLeftPower1,frontRightPower1,backRightPower1;
        if (!manual){
            touch_guide_flag = false;
            if (junction_level == 2) junction_level = 3;
            linearslidets(outtake_heights[junction_level],3);


            distance= cdis.getDistance(DistanceUnit.MM);
            if (distance>500)  distance= cdis.getDistance(DistanceUnit.MM);
            if(distance>500){
                gamepad1.rumble(500);

                return;
            }
            manual=true;
            forward(0.38);
            start_time = runtime.seconds();
            time_passed = 0;
            sleep(50);

            while ( cdis.getDistance(DistanceUnit.MM) > 150 && guide_touch.getState()&&time_passed < 0.8&& opModeIsActive() )
            {
                time_passed = runtime.seconds() - start_time;
                dropheight();
            }

            if ( !guide_touch.getState()) {

                backward(0.2);
                sleep(150);
                touch_guide_flag=false;
            }
            stop_motor();

            speed_mult = 0.2;
            return;
        }


        stop_motor();
        cam.setPosition(0);
        sleep(150);
        cam_turn.setPosition(0);
        sleep(100);
       // cam.setPosition(0);
       // sleep(150);
       // cam_turn.setPosition(0);
      //  sleep(200);
        linearslidets(outtake_heights[junction_level] + outtake_up_gap, 3);
        sleep(100);
        backward(0.7);
        sleep(250);

        linearslidets(intake_heights[height_stage], 1);
        stop_motor();
        junction_level = 1;//default set to M
        has_cone = false;
        manual=false;
        speed_mult = 0.4;

    }
    private void stackcone(){
        // TODO test for the right heights and enough wait time
        forward(0.22);
        linearslidets(intake_heights[5],2);

        boolean guide_touch_flag=false;
        boolean  take_touch_flag=false;

        sleep(150);
        time_passed=0;
        start_time=runtime.seconds();
        while (!guide_touch_flag && opModeIsActive() && time_passed <1.2){
            if(!guide_touch.getState()&&abs(front_right.getVelocity())<5) guide_touch_flag=true;
            time_passed=runtime.seconds()-start_time;
        }
        stop_motor();
        linearslidets(intake_heights[0],1);
        time_passed=0;
        start_time=runtime.seconds();

        while(opModeIsActive() && !take_touch_flag && time_passed < 1.2){

            take_touch_flag=!take_touch.getState()&&(linear_slide.getCurrentPosition()>-600);
            time_passed=runtime.seconds()-start_time;
        }

        cam.setPosition(0.3);//-0.3 0.33
        // drive.setMotorPowers(0,0,0,0);
        linear_slide.setVelocity(0);
        has_cone=true;

        sleep(150);
        //current_pos = linear_slide.getCurrentPosition();
        linearslidets( outtake_heights[height_stage], 2);

        while (linear_slide.getCurrentPosition() > -700 ){
            // slidePositionTo(linear_slide.getCurrentPosition() - 500, 2800, 0);
        }
        backward(0.5);
        sleep(200);
        stop_motor();

        cam_turn.setPosition(0.43);
    }


    public void pick() {
        linearslidets(60, 2);
        cam_turn.setPosition(0);
        stop_motor();
         time_passed = 0;
         start_time = runtime.seconds();

        while (opModeIsActive() && !intake_error &&  !take_flag && time_passed < 1.0 ) {
            if (gamepad1.b||gamepad1.circle) intake_error = true;
            time_passed = runtime.seconds()-start_time;
            take_flag = !take_touch.getState() && (linear_slide.getCurrentPosition()>-100);
           // if(linear_slide.getVelocity()<2 && linear_slide.getCurrentPosition()>10) slow_speed_flag=true;

        }
        if (time_passed>=1.0) gamepad1.rumble(500);
        linear_slide.setVelocity(0);
        if (!intake_error) {
            cam.setPosition(0.3);
            sleep(150);
            has_cone = true;
            backward(0.5);
            cam_turn.setPosition(0.43);
            linearslidets(outtake_heights[1] , 1);
        }
        backward(0.5);
        linearslidets(outtake_heights[1] , 1);
        slow_speed_flag=false;
        take_flag = false;
        intake_error = false;
        sleep(100);
        stop_motor();
    }

// spinturn








    public void spint(int counter) {
        if(junction_level==2) {
            junction_level=3;
            linearslidets(outtake_heights[junction_level],3 );
        }

        int cdist1=400;
        int cdist2 = 600;
        double spower = 0.3;
        if (junction_level == 0)spower = 0.27;
        else if (junction_level == 1) spower = 0.27;
        else if (junction_level >= 2) spower = 0.4;


        double dist=700;

        if (counter == 1) driveturn(counter, 0.6);
        else driveturn(counter, 0.6);
        cam_turn.setPosition(0.43);
        start_time = runtime.seconds();
        time_passed=0;
        while ((dist>cdist1||dist<80 )&& opModeIsActive()&& time_passed< 0.5) {
            if (counter == -1) dist = left_distance.getDistance(DistanceUnit.MM);
            else dist = right_distance.getDistance(DistanceUnit.MM);
            telemetry.addData("t0",dist);

            time_passed=runtime.seconds()-start_time;
        }
        if (counter == 1) driveturn(counter,0.36);
        else driveturn(counter,spower);
        dist=cdis.getDistance(DistanceUnit.MM);
        start_time = runtime.seconds();
        time_passed=0;
        while ((dist>cdist2 ||dist<80)&& opModeIsActive()&& time_passed< 0.4) {
            dist = cdis.getDistance(DistanceUnit.MM);
            telemetry.addData("t1",dist);

            time_passed=runtime.seconds()-start_time;
        }
        stop_motor();
         sleep(100);
        dist=cdis.getDistance(DistanceUnit.MM);
        if(dist>500) {
            driveturn(-counter,0.35);
            sleep(50);
        }
        stop_motor();
        telemetry.addData("tfinal",dist);;
        telemetry.update();
        //sleep(20000);
        // sleep(10000000);
        distance_outtake2();

    }


    public void linear_sensor_reset() {

        stop_motor();
        cam.setPosition(0);
        cam_turn.setPosition(0);
        has_cone = false;
        backward(0.6);
        sleep(150);
        stop_motor();
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        time_passed=0;
        start_time=runtime.seconds();
        linear_slide.setPower(0.5);
        while (opModeIsActive() && linear_reset.getState()&&(time_passed<1.5)) {

            time_passed=runtime.seconds()-start_time;

        }
        linear_slide.setPower(0);
        sleep(100);
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setTargetPosition(intake_heights[5]);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        height_stage = 1;
        linearslidets(intake_heights[height_stage], 3);
    }
    private void dropheight(){
        if(!has_cone) return;
        if (gamepad2.a) {
            junction_level = 0;
            linearslidets(outtake_heights[junction_level], 3);
        }
        if (gamepad2.b) {
            junction_level = 1;
            linearslidets(outtake_heights[junction_level], 3);
        }
        if (gamepad2.y) {
            junction_level = 2;
            if (manual) junction_level = 3;
            linearslidets(outtake_heights[junction_level], 3);
        }

    }




    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        // handle = hardwareMap.get(DcMotor.class, "handle");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
      //  front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        linear_slide = hardwareMap.get(DcMotorEx.class, "linear_slide");
        cam = hardwareMap.get(Servo.class, "cam");
        cam_turn=hardwareMap.get(Servo.class, "cam_turn");
        take_touch = hardwareMap.get(DigitalChannel.class, "take_touch");
        guide_touch = hardwareMap.get(DigitalChannel.class, "guide_touch");
        linear_reset = hardwareMap.get(DigitalChannel.class, "linear_reset");
        right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");
//        cam_distance = hardwareMap.get(DistanceSensor.class, "cam_distance");
        left_distance = hardwareMap.get(DistanceSensor.class, "left_distance");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        cdis = hardwareMap.get(DistanceSensor.class, "cdis");
        color = hardwareMap.get(ColorSensor.class,"color");
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //added
        // imu = hardwareMap.get(Gyroscope.class, "imu");



        //   double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
        //   double rx = -gamepad1.left_stick_x * 0.75;
        //   double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        //   double frontLeftPower = (y + x + rx) / denominator;
        //  double backLeftPower = (y - x + rx) / denominator;/  double frontRightPower = (y - x - rx) / denominator;
        //  double backRightPower = (y + x - rx) / denominator;

        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //  imuParameters.calibrationDataFile=
        // Disable logging.
        imuParameters.loggingEnabled = false;
        cdis.resetDeviceConfigurationForOpMode();
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

        drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(startPoseA);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setTargetPosition(intake_heights[1]);
        waitForStart();
        runtime.reset();
        cam.setPosition(0);
        cam_turn.setPosition(0);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearslidets(intake_heights[1], 3);
        stop_motor();
        drive.update();

        while (opModeIsActive()) {

            // Driver controls from Game Manual 0

            height_stage = 1;
           double y = gamepad1.right_stick_y; // Remember, this is reversed!
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.left_stick_x * 0.75;
           double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
           double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
           double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            speed_mult=default_speed;
            if (manual) speed_mult=0.2;



            front_left.setPower(-frontLeftPower*speed_mult);
            front_right.setPower(-frontRightPower*speed_mult);
            rear_left.setPower(-backLeftPower*speed_mult);
            rear_right.setPower(-backRightPower*speed_mult);




         // telemetry.addData("gamepad1leftstickx",gamepad1.left_stick_x);
          //  telemetry.addData("gamepad1righttickx",gamepad1.right_stick_x);
          //  telemetry.update();

            if (gamepad1.left_trigger > 0.3){
                default_speed = 0.75;
                telemetry.addData("speed change",default_speed);
                telemetry.update();
            }
            if (gamepad1.right_trigger > 0.3) {
                default_speed = 0.4;
                telemetry.addData("speed change",default_speed);
                telemetry.update();
            }


            if ( (gamepad1.y || gamepad1.x || gamepad1.square || gamepad1.triangle) && has_cone){
                distance_outtake2();
            }

//            telemetry.addData("Current Cam Distance", cdis.getDistance(DistanceUnit.MM));
//            // telemetry.addData("Velocity", front_right.getVelocity());
//            //   telemetry.addData("Current Right Distance",right_distance.getDistance(DistanceUnit.MM));
//            telemetry.update();




            dropheight();

            if (gamepad2.x) {
                stop_motor();
                cam.setPosition(0);
                cam_turn.setPosition(0);
                has_cone = false;
                sleep(100);
                linearslidets(linear_slide.getCurrentPosition() + outtake_up_gap, 3);
                sleep(150);
                backward(0.6);
                sleep(100);
                stop_motor();
                // sleep(100);
                linearslidets(intake_heights[height_stage], 2);
            }

            // linear slide goes down slowly until touch sensor is pressed check? right?
            // if ((gamepad2.dpad_right && !has_cone) || (!has_cone && !guide_touch.getState()))

            if ((gamepad2.dpad_right || !guide_touch.getState()) && !has_cone )
                pick();

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
//                spin2(-1);
                spint(-1);
            }
            if (gamepad1.right_bumper&&has_cone) {
//                spin2(1);
                spint(1);
            }
            if(gamepad2.left_bumper) stackcone();
            // telemetry.addData("Right Distance",right_distance.getDistance(DistanceUnit.MM));
            // telemetry.addData("cam distance", cdis.getDistance(DistanceUnit.MM));
            // color.enableLed(true);
            //  telemetry.addData("Alpha integer",color.alpha());
            // telemetry.addData("Red integer",color.red());
            //  telemetry.addData("Green integer",color.green());
            // telemetry.addData("Blue integer",color.blue());
            //  telemetry.addData("ARGB Value",color.argb());
           // telemetry.update();




        }

    }
}