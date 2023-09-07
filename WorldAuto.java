package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name = "WorldAuto", group = "A")
@Config
public class WorldAuto extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "ABG_Model1.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "Apple",
            "Banana",
            "Grape"
    };
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AWg52rL/////AAABmdjtwukCMEIWkkw/HJ019KRqk7zvrUf22qbakKlQZyyRyVlV7TowKMdq3JsOb5hi/6txnmzwdetP6kmm8uEZsQ+aN+rldPdNnwv/Aqo21HOx0fmRykuUthzWk2QHBoS0nwRkRRCr8fdGT8h+0He56QHhsfQNsYWi0By5ECXrss+bKu8t36cgcoOqTlfBz/msn9iCWzznek+vPjkNj6hWxjg6/uqTv10wn+NlNw5V4H/G+T0M9Bgux5gkuA1VYWU9KXUN3uyVrzMXVOJzpMt1OmpJsqWZIBs7GJ64nrIlyQZbWz7SA3i4o3+uC1cAMRTGHSJmswBc/y+oh2XrwHK8E5yl3kbbazFKRCWOk8uYGHP+";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private DcMotorEx linear_slide;
    private DcMotorEx front_right;
    private Servo cam;
    private Servo cam_turn;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private DigitalChannel take_touch;
    private DigitalChannel guide_touch;
    private BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor color;
    private DistanceSensor cdis;



    BNO055IMU.Parameters imuParameters; //added

    SampleMecanumDrive drive;


    TrajectorySequence trajSeq_A_p2lt_drop = null;
    TrajectorySequence  trajSeq_Bl_p3lt_drop = null;
    TrajectorySequence trajSeq_p2l_p1l_pick  = null;
    TrajectorySequence trajSeq_p3lt_p1lt_pick = null;
    TrajectorySequence trajSeq_p3lt_Parking1 =null;
    TrajectorySequence trajSeq_p3lt_Parking2 =null;
    TrajectorySequence trajSeq_p3lt_Parking3 =null;
    TrajectorySequence trajSeq_p3lt_Parking1e =null;
    TrajectorySequence trajSeq_p3lt_Parking2e =null;
    TrajectorySequence trajSeq_p3lt_Parking3e =null;

    TrajectorySequence trajSeq_A_p2rt_drop = null;
    TrajectorySequence  trajSeq_Br_p3rt_drop = null;
    TrajectorySequence trajSeq_p2r_p1r_pick  = null;
    TrajectorySequence trajSeq_p3rt_p1rt_pick = null;
    TrajectorySequence trajSeq_p3rt_Parking1 =null;
    TrajectorySequence trajSeq_p3rt_Parking2 =null;
    TrajectorySequence trajSeq_p3rt_Parking3 =null;
    TrajectorySequence trajSeq_p3rt_Parking1e =null;
    TrajectorySequence trajSeq_p3rt_Parking2e =null;
    TrajectorySequence trajSeq_p3rt_Parking3e =null;


    boolean left_side=true;
    boolean blue_color=true;
    boolean High=true;
    int defaultParkingzone = 2; // default

  //  double tick_length=0.7837653;


    int[] pk_height={-200,-335,-455,-575,-695};
    int[] pkl_height={-220,-300,-400,-500,-600};
    int cycles=0;
    double gyro = 0;
    double time_passed=0;
    boolean  guide_touch_flag= false;
    boolean take_touch_flag=false;
    double dist;
    int outtake_up_gap=-330;

    Pose2d clocation;
    int pk_level=4;
    int junction_level=0;
    double start_time = 0;
    int[] outtake_heights = {-990, -1860,-2400, -2650};//4 height.  the last 2 for the top height.
    Pose2d restartBl= new Pose2d(-49, -27, Math.toRadians(-90));
    Pose2d restartBr= new Pose2d(-49, 27, Math.toRadians(90));
    boolean error =false;


    // TODO adjust the move distance for the drop
    double forwardDropDistance = 5.5; // 9-10
    double dropShift = forwardDropDistance * Math.sqrt(2)/2;

    Pose2d startPoseA = new Pose2d();

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPoseA);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        linear_slide = hardwareMap.get(DcMotorEx.class, "linear_slide");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");

        cam = hardwareMap.get(Servo.class, "cam");
        cam_turn = hardwareMap.get(Servo.class, "cam_turn");
        take_touch = hardwareMap.get(DigitalChannel.class, "take_touch");
        guide_touch = hardwareMap.get(DigitalChannel.class, "guide_touch");
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //added
//        cam_distance = hardwareMap.get(DistanceSensor.class, "cam_distance");
     //   right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");
        color = hardwareMap.get(ColorSensor.class,"color");
//        guide_touch = hardwareMap.get(DigitalChannel.class, "guide_touch");
//        take_touch.setMode(DigitalChannel.Mode.INPUT);
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cam.setPosition(0);
       cam_turn.setPosition(0.1);
        buildTrajectories();
        imuParameters = new BNO055IMU.Parameters();
        cdis = hardwareMap.get(DistanceSensor.class, "cdis");
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        cdis.resetDeviceConfigurationForOpMode();
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // gyro = angles.firstAngle;
        telemetry.addLine("Driver      X/Square-Left         B/Circle-Right ");
        telemetry.addLine("Driver      A/Cross-M           Y/Triangle-H ");
        telemetry.addLine("Gunner     X-Blue         B-Red");



        //telemetry.addData("Attach the cone now");
        telemetry.update();
        // color.enableLed(false);
        left_side=true; High=true; blue_color=true;// only for debug, remove later;
        // gamestart
        while(true){
            if(gamepad1.b||gamepad1.circle) {
                left_side=false;
                telemetry.addLine("RIGHT SIDE SELECTED");
                telemetry.addLine();
                telemetry.addLine("Driver      X/Square-Left         B/Circle-Right ");
                telemetry.addLine("Driver      A/Cross-M           Y/Triangle-H ");
                telemetry.addLine("Gunner     X-Blue         B-Red");
                telemetry.update();
            }
            if(gamepad1.x||gamepad1.square) {
                left_side=true;
                telemetry.addLine("LEFT SIDE SELECTED");
                telemetry.addLine();
                telemetry.addLine("Driver      X/Square-Left         B/Circle-Right ");
                telemetry.addLine("Driver      A/Cross-M           Y/Triangle-H ");
                telemetry.addLine("Gunner     X-Blue         B-Red");
                telemetry.update();
            }
            if(gamepad1.a||gamepad1.cross) {
                High=false;
                telemetry.addLine("M Mode SELECTED");
                telemetry.addLine();
                telemetry.addLine("Driver      X/Square-Left         B/Circle-Right ");
                telemetry.addLine("Driver      A/Cross-M           Y/Triangle-H ");
                telemetry.addLine("Gunner     X-Blue         B-Red");
                telemetry.update();
            }
            if(gamepad1.y||gamepad1.triangle) {
                High=true;
                telemetry.addLine("H Mode SELECTED");
                telemetry.addLine();
                telemetry.addLine("Driver      X/Square-Left         B/Circle-Right ");
                telemetry.addLine("Driver      A/Cross-M           Y/Triangle-H ");
                telemetry.addLine("Gunner     X-Blue         B-Red");
                telemetry.update();
            }

            if(gamepad2.x) {
                blue_color=true;
                telemetry.addLine("BLUE COLOR SELECTED");
                telemetry.addLine();
                telemetry.addLine("Driver      X/Square-Left         B/Circle-Right ");
                telemetry.addLine("Driver      A/Cross-M           Y/Triangle-H ");
                telemetry.addLine("Gunner     X-Blue         B-Red");
                telemetry.update();
            }
            if(gamepad2.b) {
                blue_color=false;
                telemetry.addLine("RED COLOR SELECTED");
                telemetry.addLine();
                telemetry.addLine("Driver      X/Square-Left         B/Circle-Right ");
                telemetry.addLine("Driver      A/Cross-M           Y/Triangle-H ");
                telemetry.addLine("Gunner     X-Blue         B-Red");
                telemetry.update();
            }
            if (!take_touch.getState()){
                cam.setPosition(0.30);
                if(left_side)   telemetry.addLine("LEFT");
                else   telemetry.addLine("RIGHT");
                if(blue_color)    telemetry.addLine("BLUE");
                else  telemetry.addLine("RED");
                if(High)    telemetry.addLine("H ROUTE");
                else  telemetry.addLine("M ROUTE");
                break;
            }
        }
        // initialize camera to get ready
        initVuforia();
        tfod = initTfod();
        if (tfod == null) {
            telemetry.addData(">", "initTfod failed");
            telemetry.update();
        } else {
            // Activate TensorFlow Object Detection before we wait for the start command.
            tfod.activate();
            // zoom can be adjustaed, it seems 1.0 is the best
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        // when competition, comment out FtcDashboard related
        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        /** Wait for the game to begin */
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start op mode ");
        telemetry.update();


        waitForStart();
        runtime.reset();
        cam_turn.setPosition(0.2);
        // slide up asynchronously
        startSlideUp(true);
        if (isStopRequested())
            return;


        while (opModeIsActive() && !isStopRequested()) {
            int parkingZone = getParkingZoneFromPic();

            autoDriveDropAndPark(parkingZone);
            sleep(300);
            break;
        }
    }
    private void startSlideUp(boolean asyn){
        if(asyn) {
            Thread thread = new Thread() {
                public void run() {
                    startSlideUp();
                }
            };
            thread.start();
        }else
            startSlideUp();
    }
private void cycle0 () {
// cylle0 is the first cyble  from start point to drop the first cone and ready to pick second
    if(left_side) {
        drive.followTrajectorySequence( trajSeq_A_p2lt_drop);
        spina(1);
        stop_motor();
        drive.followTrajectorySequence(trajSeq_p2l_p1l_pick);
    }
    else{
        drive.followTrajectorySequence( trajSeq_A_p2rt_drop);
        spina(-1);
        stop_motor();
        drive.followTrajectorySequence(trajSeq_p2r_p1r_pick);
    }
}
private void cycle1 (boolean hi) { // cycle 1 is the repeatbable cycles after cycle0.
        junction_level=2;
        if (!hi) junction_level = 1; // Hi=true means all the rest target are high junction.
        pickCone();
        if(error) return;
        cycles++;
     if(left_side){
         drive.followTrajectorySequence(trajSeq_Bl_p3lt_drop);
         if(hi) {
             spina(1);
             sleep(100);
             // stop_motor();
             driveturn(-1,1.0);
         }
         else {
             spina(-1);
             sleep(100);
             // stop_motor();
             driveturn(1,1.0);
         }
     }
     else{
         drive.followTrajectorySequence(trajSeq_Br_p3rt_drop);
         if(hi) {
             spina(-1);
             sleep(100);
             // stop_motor();
             driveturn(1,1.0);
         }
         else {
             spina(1);
             sleep(100);
             // stop_motor();
             driveturn(-1,1.0);
         }

     }//right side


        sleep (150);
        stop_motor();
       // sleep(20);
    }

private void park(int zone)   //final parking .
{
    linearslidets(0,2400);
    if (left_side&&!error) {
        if (zone == 1) {
            drive.followTrajectorySequence( trajSeq_p3lt_Parking1);
            return;
        }
        if (zone == 2) {
            drive.followTrajectorySequence( trajSeq_p3lt_Parking2);
            return;
        }
        drive.followTrajectorySequence( trajSeq_p3lt_Parking3);
        return;
    }
    if (!left_side&&!error){
        if (zone == 1) {
            drive.followTrajectorySequence( trajSeq_p3rt_Parking1);
            return;
        }
        if (zone == 2) {
            drive.followTrajectorySequence( trajSeq_p3rt_Parking2);
            return;
        }
        drive.followTrajectorySequence( trajSeq_p3rt_Parking3);
        return;
    }

    if (left_side&&error) {
        if (zone == 1) {
            drive.followTrajectorySequence( trajSeq_p3lt_Parking1e);
            return;
        }
        if (zone == 2) {
            drive.followTrajectorySequence( trajSeq_p3lt_Parking2e);
            return;
        }
        drive.followTrajectorySequence( trajSeq_p3lt_Parking3e);
        return;
    }

    if (!left_side&&error){
        if (zone == 1) {
            drive.followTrajectorySequence( trajSeq_p3rt_Parking1e);
            return;
        }
        if (zone == 2) {
            drive.followTrajectorySequence( trajSeq_p3rt_Parking2e);
            return;
        }
        drive.followTrajectorySequence( trajSeq_p3rt_Parking3e);
        return;
    }
}


    private void startSlideUp(){
        // slide up initially
        linear_slide.setTargetPosition(outtake_heights[junction_level]);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear_slide.setTargetPosition(outtake_heights[junction_level]);
        linear_slide.setVelocity(1600);
        //sleep(400);
    }

    private void slideUpAsyncTo(int targetPosition){
        Thread thread = new Thread(){
            public void run(){
                linear_slide.setTargetPosition(targetPosition);
                linear_slide.setVelocity(2200);
            }
        };
        thread.start();
    }

    /**
     * Define the point coordinates, and build each movement between pick and drop
     */
    private void buildTrajectories(){


        Pose2d p1lt=new Pose2d(-46, -24,Math.toRadians(-90));
        Pose2d  p1l = new Pose2d(-47, -22, Math.toRadians(-90));//-49.5,-23.5//-48,-22
        Pose2d  p3lt = new Pose2d(-49, 18, Math.toRadians(-90));
        Pose2d  p2l = new Pose2d(-49, 0, Math.toRadians(-45));

        Pose2d p1rt=new Pose2d(-47, 22,Math.toRadians(90));
        Pose2d  p1r = new Pose2d(-49.5, 21, Math.toRadians(90));//-49.5,-23.5//-48,-22
        Pose2d  p3rt = new Pose2d(-49, -20, Math.toRadians(90));
        Pose2d  p2r = new Pose2d(-49, 0, Math.toRadians(45));

        Pose2d p1rPose = new Pose2d(-47, -26, Math.toRadians(90));
        Pose2d p2rPose = new Pose2d(-48, 0, Math.toRadians(90));
        Pose2d p3rPose = new Pose2d(-48, 23, Math.toRadians(90));

        Pose2d p1Pose = new Pose2d(-47, -24, Math.toRadians(-90));
        Pose2d p2Pose = new Pose2d(-47, -1.5, Math.toRadians(-90));
        Pose2d p3Pose = new Pose2d(-48, 24, Math.toRadians(-90));



        trajSeq_A_p2lt_drop = drive.trajectorySequenceBuilder(startPoseA)
                //.lineToSplineHeading(p2lt)
                .back(53)
                .build();

        trajSeq_Bl_p3lt_drop = drive.trajectorySequenceBuilder(restartBl)
                .back(47)
                .build();

        trajSeq_p2l_p1l_pick = drive.trajectorySequenceBuilder(p2l)
                .lineToSplineHeading(p1l)
                .build();
        trajSeq_p3lt_p1lt_pick = drive.trajectorySequenceBuilder(p3lt)

                //  .forward(40)
                .lineToLinearHeading(p1lt)// line to spline heading
                .build();


        trajSeq_p3lt_Parking1 = drive.trajectorySequenceBuilder(p3lt)
                .lineToLinearHeading(p1Pose)
                .build();

        trajSeq_p3lt_Parking2 = drive.trajectorySequenceBuilder(p3lt)
                .lineToLinearHeading(p2Pose)
                .build();
        trajSeq_p3lt_Parking3 = drive.trajectorySequenceBuilder(p3lt)
                .lineToLinearHeading(p3Pose)
                .build();

        trajSeq_p3lt_Parking1e = drive.trajectorySequenceBuilder(p1lt)
                .lineToLinearHeading(p1Pose)
                .build();

        trajSeq_p3lt_Parking2e = drive.trajectorySequenceBuilder(p1lt)
                .lineToLinearHeading(p2Pose)
                .build();
        trajSeq_p3lt_Parking3e = drive.trajectorySequenceBuilder(p1lt)
                .lineToLinearHeading(p3Pose)
                .build();


        /// the right
        trajSeq_A_p2rt_drop = drive.trajectorySequenceBuilder(startPoseA)
                //.lineToSplineHeading(p2lt)
                .back(53)
                .build();

        trajSeq_p2r_p1r_pick = drive.trajectorySequenceBuilder(p2r)
                .lineToSplineHeading(p1r)
                .build();

        trajSeq_Br_p3rt_drop = drive.trajectorySequenceBuilder(restartBr)
                . lineToLinearHeading(p3rt)
                .build();


        trajSeq_p3rt_p1rt_pick = drive.trajectorySequenceBuilder(p3rt)

                //  .forward(40)
                .lineToLinearHeading(p1rt)// line to spline heading
                .build();

        trajSeq_p3rt_Parking1e = drive.trajectorySequenceBuilder(p1rt)
                .lineToLinearHeading(p1rPose)
                .build();

        trajSeq_p3rt_Parking2e = drive.trajectorySequenceBuilder(p1rt)
                .lineToLinearHeading(p2rPose)
                .build();
        trajSeq_p3rt_Parking3e = drive.trajectorySequenceBuilder(p1rt)
                .lineToLinearHeading(p3rPose)
                .build();




        trajSeq_p3rt_Parking1 = drive.trajectorySequenceBuilder(p3rt)
                .lineToLinearHeading(p1rPose)
                .build();

        trajSeq_p3rt_Parking2 = drive.trajectorySequenceBuilder(p3rt)
                .lineToLinearHeading(p2rPose)
                .build();
        trajSeq_p3rt_Parking3 = drive.trajectorySequenceBuilder(p3rt)
                .lineToLinearHeading(p3rPose)
                .build();
    }

    private void autoDriveDropAndPark(int parkingZone) {
//main program

        cycle0();
        while (runtime.seconds() < 23) {
            cycle1(High);
            if (runtime.seconds() > 22||error) break;
            telemetry.addData("time",runtime.seconds());
            telemetry.update();
          if(left_side)  drive.followTrajectorySequence(trajSeq_p3lt_p1lt_pick);
          else {
              drive.followTrajectorySequence(trajSeq_p3rt_p1rt_pick);
          }


        }
        park(parkingZone);
        stop_motor();
    }
//



    public void spina(int counter) {
        driveturn(counter, 1);
        cam_turn.setPosition(0.43);

        // the linearslides will not raise the target height until turning to the junction to increase teh stability
        if(junction_level==2) {
            junction_level=3;

        }
        linearslidets(outtake_heights[junction_level], 2600);
        int cdist=400;
        int qt=115;
        double spin_power = 0.42;
        dist=600;

        if(cycles==0&&left_side ){
            cdist=550;
            qt=65;  // 82
            spin_power=0.35;// use different power for the accuracy.

        }
        if(cycles==0&&!left_side) {
            cdist=550;
            qt=65;  // 82
            spin_power=0.30;

        }
        if(cycles>0&&!left_side){
            qt = 130;
            spin_power=0.45;
        }


        sleep(qt);//check
        start_time = runtime.seconds();
        time_passed=0;
        driveturn(counter, spin_power);
            while ((dist>cdist ||dist<80)&& opModeIsActive()&& time_passed< 0.4) {
                dist = cdis.getDistance(DistanceUnit.MM);
                telemetry.addData("t",dist);

                time_passed=runtime.seconds()-start_time;
            }
            stop_motor();
       // location();
        dist=cdis.getDistance(DistanceUnit.MM);
        telemetry.addData("t2",dist);
        telemetry.update();
        stop_motor();

        if(dist>550) {
            driveturn(-counter,0.4);
            sleep(70);
        }
      // sleep(10000000);
        distance_outtake2();

    }

    private double abs(double input) {
        if (input >= 0) return (input);
        else
            return (-input);
    }

    private void linearslidets(int target, int speed) {
        linear_slide.setTargetPosition(target);
        linear_slide.setVelocity(speed);

    }




    public void straft (int left ,double power)

    {
        drive.setMotorPowers(-power*left, power*left, -power*left, power*left);
    }

    public void stop_motor()
    {drive.setMotorPowers(0, 0, 0, 0);}
    public void forward(double power)
    {drive.setMotorPowers(power, power, power, power);}


    public void driveturn(int counter, double power) {
        drive.setMotorPowers(power*counter, power*counter, -power*counter, -power*counter);
    }


    public void distance_outtake2( ) {

        // double drive_distance=200;
      //  cam_turn.setPosition(0.43);
    //check
        cam.setPosition(0.2);

        start_time = runtime.seconds();
        time_passed = 0;

        forward(0.40);
        if(junction_level==0) forward(0.35);

        while ( cdis.getDistance(DistanceUnit.MM) > 155  &&time_passed < 0.6&& opModeIsActive() )
        {
            time_passed = runtime.seconds() - start_time;

        }

        stop_motor();
        cam.setPosition(0);
        if (cycles == 0) sleep(50);
        sleep(150);
        cam_turn.setPosition(0);
        if (cycles == 0) sleep(50);
        sleep(150);
        linearslidets(outtake_heights[junction_level] + outtake_up_gap, 2600);
        sleep(100);
       if(junction_level==3) forward(-0.5);
       else forward(-0.8);
        sleep(220);
        linearslidets(pk_height[pk_level],1600);
       // forward(-0.6);

    }

    private void moveforward(double DISTANCE) {

        // clocation = drive.getPoseEstimate();
        drive.update();
        clocation = drive.getPoseEstimate();
        Trajectory move = drive.trajectoryBuilder(clocation)
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
       // telemetry.update();

    }

    private void coloralign () { // make alignment with color tape on the ground.

        int dir=-1;
        int bhigh, rhigh,cvalue,chigh;
        bhigh=340;
        rhigh=235;

        if(!left_side) dir=1;
        if(blue_color) {
            chigh = bhigh;
            cvalue= color.blue();
        }
        else{
            chigh=rhigh;
            cvalue= color.red();
        }

        if (cvalue>chigh) return;

       straft(dir,0.24);
        start_time=runtime.seconds();
        time_passed=0;
        while ( !error && opModeIsActive()&& time_passed<1.4) {
            if (time_passed > 0.9)
                straft(-dir,0.3);

            if(blue_color) cvalue= color.blue();
            else cvalue= color.red();


            if (cvalue>chigh) return;
            time_passed=runtime.seconds()-start_time;
        }
        if (time_passed> 1.5) {
            error= true;
            straft(dir,0.3);
            sleep(300);
            stop_motor();

        }
    }



    private void pickCone(){
        double angle_gap;
        guide_touch_flag=false;
        take_touch_flag=false;
        stop_motor();
        coloralign();
        stop_motor();
        gyro=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        angle_gap=abs(90-abs(gyro));
// make sure the heading is vertical to the wall.
        if (left_side&&angle_gap>3)
            drive.turn(Math.toRadians(-90-gyro));
        if (!left_side&&angle_gap>3)
            drive.turn(Math.toRadians(90-gyro));
        stop_motor();
        if(error) return;
        forward(0.25);
        sleep(200);
        forward(0.2);
       start_time=runtime.seconds();
       time_passed=0;
        while (!guide_touch_flag && opModeIsActive() && time_passed <1.2){
            if(!guide_touch.getState()&&cdis.getDistance(DistanceUnit.MM)<30) guide_touch_flag=true;
            time_passed=runtime.seconds()-start_time;
        }
        if (time_passed > 1.2) {
           stop_motor();
           error = true;
           return;
        }

        stop_motor();
       linearslidets(-50,2000);
        if (left_side) drive.setPoseEstimate(restartBl);// reset  the location for calibration
        else drive.setPoseEstimate(restartBr);/// reset  the location for  calibration

        start_time=runtime.seconds();
        time_passed=0;
        while(opModeIsActive() && !take_touch_flag && time_passed < 1.2){

            take_touch_flag=!take_touch.getState()&&(linear_slide.getCurrentPosition()>-650);
            time_passed=runtime.seconds()-start_time;
        }
        if(time_passed>1.2)  {
            error=true;
            linearslidets(outtake_heights[junction_level],2600 );
            sleep(500);
            stop_motor();
            return;
        }


        if (left_side) drive.setPoseEstimate(restartBl);// calibrate the location.
        else drive.setPoseEstimate(restartBr);

        cam.setPosition(0.30);
        linear_slide.setVelocity(0);

        sleep(100);
     linearslidets(outtake_heights[junction_level],2600 );

        while (linear_slide.getCurrentPosition() > pkl_height[pk_level]){

        }
        pk_level--; // lower the pick up height form the cones pile
    }

//    /**
//     * Check position for the waiting time - seems not that accurate
//     * @param targetPosition
//     * @param moveLower
//     */
//    private void waitSlideToPosition(int targetPosition, boolean moveLower){
//        int currentPosition = linear_slide.getCurrentPosition();
//        boolean reached = moveLower? currentPosition > targetPosition:currentPosition<targetPosition;
//
//        while(!reached && Math.abs(currentPosition - targetPosition)>20) {
//            sleep(50);
//            currentPosition = linear_slide.getCurrentPosition();
//            reached = moveLower? currentPosition > targetPosition:currentPosition<targetPosition;
//        }
//        sleep(50);
//    }

    /**
     * move slide to a position, given an option to wait or not for the slide to reach the position
     * @param targetPosition
     * @param velocity
     * @param timeout 0 means no timeout
     */



    private void slidePositionTo(int targetPosition, double velocity, long timeout){
        linear_slide.setTargetPosition(targetPosition);
        linear_slide.setVelocity(velocity);

        if(timeout > 0) {
            sleep(timeout);
        }
    }


    private void deactivateTFOD(){
        Thread t = new Thread(){
            public void run(){
                if(tfod != null)
                    tfod.deactivate();
            }
        };
        t.start();
    }

    private int getParkingZoneFromPic(){
        int parkingzone = defaultParkingzone;
        long start = System.currentTimeMillis();
        boolean toDeactivated = true;
        while (opModeIsActive() && !isStopRequested() && tfod != null && (System.currentTimeMillis() - start) <300) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && !updatedRecognitions.isEmpty()) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                String objLabel = updatedRecognitions.get(0).getLabel();
                if(objLabel != null) {
                    if (objLabel.equals("Apple")) {
                        parkingzone = 1;
                    } else if (objLabel.equals("Banana")) {
                        parkingzone = 2;
                    } else if (objLabel.equals("Grape")) {
                        parkingzone = 3;
                    }
                }
                telemetry.addData("Image: ", objLabel==null?"null":objLabel);
                telemetry.addData("Parking zone", "" + parkingzone);
                telemetry.update();
                deactivateTFOD();
                toDeactivated = false;
                break; // found it
            }
        }
        if(toDeactivated)
            deactivateTFOD();

        return parkingzone;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private TFObjectDetector initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        try {
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        }catch (Exception e){
            return null;
        }
        return tfod;
    }
}
