package pedroPathing.autonomous;


import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.NanoTimer;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "RIGHT AUTO", group = "AUTO")
/// 5+0 for state
public class right_auto_new extends OpMode {

    private Follower follower;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;


    private Servo servo_outtake_flip1, servo_outtake_flip2, servo_intake_wrist, servo_intake_rotate, sweeper, servo_intake, servo_outtake, servo_outtake_rotate;
    private DcMotorEx up1, up2, out;
    private TouchSensor up_zero, out_zero;


    boolean outisclosed = false;

    boolean inisclosed = false;
    // Timers
    private Timer opmodeTimer;
    private NanoTimer pathTimer; // Timers for progression of states, in nano seconds

    // Important Pose parts
    double pickup_x = 0.6; // Distance from the claw to the sample
    double turn_distance_x = 20; // Distance to start pickup
    double pickups_y = 41.00; // Observation Zone pickup distance
    private int pathState, armState, outclawState, outgrabState; // Different cases and states of the different parts of the robot

    // Poses, and pickUp Poses
    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO
    private Pose readyPose = new Pose(turn_distance_x,pickups_y, Math.toRadians(180)); // turn spot so make sure it would be safe
    private Pose pickupPose = new Pose(pickup_x, pickups_y, Math.toRadians(180));
    private Pose pickupPose1 = new Pose(pickup_x,18,Math.toRadians(180)); // first pickup poses

    /// hang poses
    private Pose hangPose = new Pose(35, 74, Math.toRadians(0)); // Hits it pretty hard, but its fine
    private Pose firsthangPose = new Pose(36.1,71,0); // Quick hang, then back
    private Pose secondhangPose = new Pose(36.1,65,0); // Should be quick hang, but idk they all mess up around here but somehow everything just works fine
    private Pose thirdhangPose = new Pose(36.3, 72,0); // Last hang

    /// push poses  for case transitions
    private Pose pushstart = new  Pose(59,30,Math.toRadians(180)); // Bezier curve end behind second sample point
    private Pose firstpushPose = new Pose(20,30, Math.toRadians(0)); // ^^ only x matters, keep y same
    private Pose pushstart2 = new Pose(59,18,Math.toRadians(180));  // same

    /// Paths, and path chains : pushFirst and pushSecond are called after hangFirst
    private Path hang_First, pickup, first_hang, first_hang_back, second_hang, second_hang_back, third_hang, park;
    private PathChain pushFirst, pushSecond;

    // Motors
    private Telemetry telemetryA;

    // variables
    int up_hanging_position = 1778; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1559; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.

    public void buildPaths() {
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 2: //go to hang

        }
    }
    public void autonomousActionUpdate() {
            switch (armState) {
                case -1: // default stop


            }
        switch (outclawState) {
            case -1: // Init Pos

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setArmState(int aState) {
        armState = aState;
    }
    public void setoutGrabState(int gstate) {
        outgrabState = gstate;
    }
    public void setoutClawState(int cState) {
        outclawState = cState;
    }


    @Override
    public void loop() {

        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();


        // Feedback to Driver Hub, states and timers
        telemetryA.addData("path state", pathState);
        telemetryA.addData("arm state", armState);
        telemetryA.addData("claw state", outclawState);
        telemetryA.addData("out grab state", outgrabState);
        telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());



        // Poses, follower error
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("Follower busy", follower.isBusy());
        telemetryA.addData("headingPID error", follower.headingError);
        telemetryA.update();
    }




        // We init the timers, telemetry, motors , and follower
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();

        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //from rr version

        //setup arm to use velocity
        //setup arm variable
        up1 = hardwareMap.get(DcMotorEx.class, "up1");
        up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up1.setDirection(DcMotorSimple.Direction.REVERSE);

        up2 = hardwareMap.get(DcMotorEx.class, "up2");
        up2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo_intake = hardwareMap.get(Servo.class, "intake");
        servo_intake_rotate = hardwareMap.get(Servo.class, "intakeRotate");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");


        servo_outtake_flip1 = hardwareMap.get(Servo.class, "ofip1");
        servo_outtake_flip2 = hardwareMap.get(Servo.class, "ofip2");

        servo_outtake = hardwareMap.get(Servo.class, "outtake");
        servo_outtake_rotate = hardwareMap.get(Servo.class, "outtaker");

        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");

        Pose startPose = new Pose(15, 40.0, Math.toRadians(0)); //TODO
        follower.setStartingPose(startPose);

        inisclosed = false;

        outisclosed = false;


    }

    @Override
    public void init_loop() {


        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetryA.addData("ready to cook cook cook cook cook", "Finished");
        }
    }
    @Override
    public void start() {
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(2);
        setArmState(-1);
        setoutGrabState(-1);
        setoutClawState(-1);
    }
    @Override
    public void stop() {
    }
}