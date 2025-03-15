package pedroPathing.autonomous;


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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "LEFT AUTO", group = "AUTO")

public class left_auto extends OpMode {

    private Follower follower;

    // Timers
    private Timer opmodeTimer;
    private NanoTimer pathTimer; // Timers for progression of states, in nano seconds

    // Important Pose parts
    /*double pickup_x = 0.6; // Distance from the claw to the sample
    double turn_distance_x = 20; // Distance to start pickup
    double pickups_y = 41.00; // Observation Zone pickup distance*/

    //Initializing the states for the robot
    private int pathState, upArmState, outArmState, intakeState, outtakeState; // Different cases and states of the different parts of the robot

    // Poses, and pickUp Poses
    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO
    private Pose bucketPose = new Pose(15, 129, Math.toRadians(135)); //TODO

    /// Paths, and path chains : pushFirst and pushSecond are called after hangFirst

    private Path preload, pickup1, basket1, pickup2, basket2, pickup3, basket3, park;

    /// Motors
    private Servo servo_outtake_flip1, servo_outtake_flip2, servo_intake_wrist, servo_intake_rotate, sweeper, servo_intake, servo_outtake, servo_outtake_rotate;
    private DcMotorEx up1, up2, out;
    private TouchSensor up_zero, out_zero;
    //private TouchSensor up_zero;
    private Telemetry telemetryA;

    /// variables
    int up_basket_position = 0; //TODO: calibrate this value, slide position to
    int outarm_in_position = 0;//TODO: Set value
    int outarm_out_position = 0;//TODO: set value
    int outarm_transfer_position = 0;//TODO: set value
    int basket_height = 2800;

    public void buildPaths() {
        // preload
        preload = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(bucketPose)
                )
        );
        preload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        pickup1 = new Path(
                // Line 2
                new BezierLine(
                        new Point(bucketPose),
                        new Point(20.000, 122.000, Point.CARTESIAN)
                )
        );
        pickup1.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0));
        basket1 = new Path(
                // Line 3
                new BezierLine(
                        new Point(20.000, 122.000, Point.CARTESIAN),
                        new Point(bucketPose)
                )
        );
        basket1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        pickup2 = new Path(
                // Line 4
                new BezierLine(
                        new Point(bucketPose),
                        new Point(26.000, 131.500, Point.CARTESIAN)
                )
        );
        pickup2.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0));
        basket2 = new Path(
                // Line 5
                new BezierLine(
                        new Point(26.000, 131.500, Point.CARTESIAN),
                        new Point(bucketPose)
                )
        );
        basket2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        pickup3 = new Path(
                // Line 6
                new BezierLine(
                        new Point(bucketPose),
                        new Point(28.000, 131.300, Point.CARTESIAN)
                )
        );
        pickup3.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(35));
        basket3 = new Path(
                // Line 7
                new BezierLine(
                        new Point(28.000, 131.300, Point.CARTESIAN),
                        new Point(bucketPose)
                )
        );
        basket3.setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(135));
        park = new Path(
                // Line 8
                new BezierCurve(
                        new Point(bucketPose),
                        new Point(58.400, 131.000, Point.CARTESIAN),
                        new Point(59.000, 95.300, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270));

    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            //FIRST BASKET/PRELOAD

            case 2: //go to basket preload sample
                follower.followPath(preload);
                setUpArmState(1);
                setOuttakeState(1);
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;
            case 3: //let go
                setOuttakeState(2);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(4);
                }
                break;

                //SECOND BASKET/FIRST PICKUP

            case 4: //go to pickup pos
                follower.followPath(pickup1);
                setOutArmState(1);
                setIntakeState(1);
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5: //actually pickup
                setIntakeState(2);
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(6);
                }
                break;
            case 6: //transfer pos
                setIntakeState(9);
                setOutArmState(9);
                setUpArmState(0); //TODO: is this right for transfer? if not also change others with same todo!
                setOuttakeState(9);
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(7);
                }
                break;
            case 7: //transfer
                setIntakeState(10);
                setOuttakeState(10);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(8);
                }
                break;
            case 8: //goto hang pos
                setOuttakeState(1);
                setOutArmState(0);
                setUpArmState(1);
                follower.followPath(basket1);
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;
            case 9:
                setOuttakeState(2);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(10);
                }
                break;

                //THIRD BASKET/SECOND PICKUP

            case 10: //go to pickup pos
                follower.followPath(pickup2);
                setOutArmState(1);
                setIntakeState(1);
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11: //actually pickup
                setIntakeState(2);
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(12);
                }
                break;
            case 12: //transfer pos
                setIntakeState(9);
                setOutArmState(9);
                setUpArmState(0); //TODO: is this right for transfer? if not also change others with same todo!
                setOuttakeState(2);
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(13);
                }
                break;
            case 13: //transfer
                setIntakeState(10);
                setOuttakeState(10);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(14);
                }
                break;
            case 14: //goto hang pos
                setOuttakeState(1);
                setOutArmState(0);
                setUpArmState(1);
                follower.followPath(basket2);
                if (!follower.isBusy()) {
                    setPathState(15);
                }
                break;
            case 15:
                setOuttakeState(2);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(16);
                }
                break;

                //FOURTH BASKET/THIRD PICKUP

            case 16: //go to pickup pos
                follower.followPath(pickup3);
                setOutArmState(1);
                setIntakeState(1);
                if (!follower.isBusy()) {
                    setPathState(17);
                }
                break;
            case 17: //actually pickup
                setIntakeState(2);
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(18);
                }
                break;
            case 18: //transfer pos
                setIntakeState(9);
                setOutArmState(9);
                setUpArmState(0); //TODO: is this right for transfer? if not also change others with same todo!
                setOuttakeState(9);
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(19);
                }
                break;
            case 19: //transfer
                intakeState = 10;
                setOuttakeState(10);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(20);
                }
                break;
            case 20: //goto hang pos
                setOuttakeState(1);
                setOutArmState(0);
                setUpArmState(1);
                follower.followPath(basket3);
                if (!follower.isBusy()) {
                    setPathState(21);
                }
                break;
            case 21:
                setOuttakeState(2);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(22);
                }
                break;

                //PARK

            case 22:
                follower.followPath(park);
                setIntakeState(0);
                setOuttakeState(0);
                setOutArmState(0);
                setUpArmState(0);

        }
    }

    public void autonomousActionUpdate() { //TODO: Ruben... Set up variables for all necessary values so it's easy to calibrate
        switch (outtakeState) {
            case 0: //flipper init pos
                servo_outtake_flip2.setPosition(1);
                servo_outtake_flip1.setPosition(0);
                servo_outtake_rotate.setPosition(0.1);
                servo_outtake.setPosition(0.1);
                break;
            case 1: //flipper basket pos but claw closed
                servo_outtake_flip2.setPosition(0.6);
                servo_outtake_flip1.setPosition(0.5);
                servo_outtake_rotate.setPosition(0.75);
                servo_outtake.setPosition(0.1);
                break;
            case 2: //flipper basket pos, claw open
                servo_outtake_flip2.setPosition(0.6);
                servo_outtake_flip1.setPosition(0.5);
                servo_outtake_rotate.setPosition(0.75);
                servo_outtake.setPosition(1);
                break;
            case 9: //flipper transfer pos, claw closed
                servo_outtake_flip2.setPosition(0.6); //TODO: THESE VALUES FOR TRANSFER!
                servo_outtake_flip1.setPosition(0.5);//TODO: THESE VALUES FOR TRANSFER!
                servo_outtake_rotate.setPosition(0.75);//TODO: THESE VALUES FOR TRANSFER!
                servo_outtake.setPosition(0.1);
                break;
            case 10: //flipper transfer pos, claw open
                servo_outtake_flip2.setPosition(0.6);//TODO: THESE VALUES FOR TRANSFER!
                servo_outtake_flip1.setPosition(0.5);//TODO: THESE VALUES FOR TRANSFER!
                servo_outtake_rotate.setPosition(0.75);//TODO: THESE VALUES FOR TRANSFER!
                servo_outtake.setPosition(1);
                break;
        }
        switch (intakeState) {
            case 0: //initialize position
                servo_intake.setPosition(0);
                servo_intake_rotate.setPosition(0.5);
                servo_intake_wrist.setPosition(0); //TODO: Please check all of the variables
                break;
            case 1: //prepare to pick up position
                servo_intake.setPosition(0.9); //FIXED: my goal is to have the claw open here
                servo_intake_rotate.setPosition(0.5); //TODO: my goal is to have the rotate straight
                servo_intake_wrist.setPosition(0.5); //TODO: my goal is to have the wrist parallel to the ground
                break;
            case 2: //robot straight pick up open position. basically means that the robot intake is in the right spot but hasn't closed the claw yet
                servo_intake.setPosition(0);
                servo_intake_rotate.setPosition(0.5);
                servo_intake_wrist.setPosition(1);
                break;
            case 3: //straight pick up closed position. same thing as above but with a closed claw
                servo_intake.setPosition(1);
                servo_intake_rotate.setPosition(0.5);
                servo_intake_wrist.setPosition(1);
                break;
            case 4: //picking up from an angle. No idea what this will look like.
                servo_intake.setPosition(1);
                servo_intake_rotate.setPosition(0.5);
                servo_intake_wrist.setPosition(1);
                break;
            case 9: //transfer position! Same as the initialization position but I created the case for clarity. If you need to edit values, you can.
                servo_intake.setPosition(0);
                servo_intake_rotate.setPosition(0.5);
                servo_intake_wrist.setPosition(0);
                break;
            case 10: //transfer position but claw open!
                servo_intake.setPosition(1);
                servo_intake_rotate.setPosition(0.5);
                servo_intake_wrist.setPosition(0);
                break;
        }
        switch (outArmState) { //All case 9s are for transfer position
            case 0: //in
                out.setTargetPosition(outarm_in_position);
                out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case 1: //out
                out.setTargetPosition(outarm_out_position);
                out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case 9:
                out.setTargetPosition(outarm_transfer_position);
                out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
        switch (upArmState) {
            case 0: //Initialization position
                up1.setTargetPosition(0);
                up2.setTargetPosition(0);
                up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case 1: //up arm position for dropping sample
                up1.setTargetPosition(basket_height);
                up2.setTargetPosition(basket_height);
                up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setOuttakeState(int otState) {
        outtakeState = otState;
    }

    public void setIntakeState(int itState) {
        intakeState = itState;
    }

    public void setOutArmState(int oaState) {
        outArmState = oaState;
    }

    public void setUpArmState(int uaState) {
        upArmState = uaState;
    }


    @Override
    public void loop() {

        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();


        // Feedback to Driver Hub, states and timers
        telemetryA.addData("path state", pathState);

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

        // Timers init
        pathTimer = new NanoTimer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Follower, and it's constants init
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Telemetry init
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        //setup arms init
        up1 = hardwareMap.get(DcMotorEx.class, "up1"); //DONE: Is this still true?
        up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up1.setDirection(DcMotorSimple.Direction.REVERSE);

        up2 = hardwareMap.get(DcMotorEx.class, "up2"); //DONE: Is this still true?
        up2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up2.setDirection(DcMotorSimple.Direction.REVERSE);

        out = hardwareMap.get(DcMotorEx.class, "out");
        int charles = out.getCurrentPosition(); //this is to set the target position to the current position, and you can use charles later to do this! (should problebly be renamed)
        out.setTargetPosition(charles);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        out.setPower(1);

        // Servos init
        servo_intake = hardwareMap.get(Servo.class, "intake");
        servo_intake_rotate = hardwareMap.get(Servo.class, "intakeRotate");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");


        servo_outtake_flip1 = hardwareMap.get(Servo.class, "ofip1");
        servo_outtake_flip2 = hardwareMap.get(Servo.class, "ofip2");

        servo_outtake = hardwareMap.get(Servo.class, "outtake");
        servo_outtake_rotate = hardwareMap.get(Servo.class, "outtaker");

        //Sensor init
        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");

    }

    /** This method is called continuously after Init while waiting for "play", it's not necessary but its useful to give more time to build paths **/
    @Override
    public void init_loop() {


        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetryA.addData("ready to cook cook cook cook cook", "Finished");
        }
    }
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        //buildPaths();
        opmodeTimer.resetTimer();
        //TODO: Change all of these values below
        setPathState(2);
        setIntakeState(0);
        setOuttakeState(0);
        setOutArmState(0);
        setUpArmState(0);


    }



    @Override
    public void stop() {
    }
}