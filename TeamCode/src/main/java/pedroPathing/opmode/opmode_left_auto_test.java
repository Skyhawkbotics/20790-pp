package pedroPathing.opmode;


import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;


import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "opmode LEF AUTO ", group = "MAIN")
public class opmode_left_auto_test extends OpMode {
    private Follower follower;

    private int outtakeState, intakeState, upArmState, outArmState;



    boolean hang = false;





    private Servo servo_outtake_flip1, servo_outtake_flip2, servo_intake_wrist, servo_intake_rotate, sweeper, servo_intake, servo_outtake, servo_outtake_rotate;
    private DcMotorEx up1, up2, out;
    private TouchSensor up_zero, out_zero;
    //from rr version of opmode_MAIN
    int arm_upper_lim = 2700;
    int up_true_target_pos;
    int out_true_target_pos;
    double servo_outtake_flip_location = 0;
    double servo_intake_wrist_location = 0.7;


    boolean goingdown = false;


    boolean defend = false;


    boolean outisclosed = false;


    boolean inisclosed = false;


    int outarm_in_position = 0;
    int outarm_out_position = 500; //TODO: Tune this value
    int outarm_transfer_position = 250; //TODO: Tune this value
    int basket_height = 2800;


    private final ElapsedTime runtime = new ElapsedTime();


    private Gamepad currentgamepad2 = new Gamepad();
    private Gamepad previousgamepad2 = new Gamepad();
    //path
    private Gamepad currentgamepad1 = new Gamepad();
    private Gamepad previousgamepad1 = new Gamepad();



    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void start() {
        //PATHING
    }


    @Override
    public void init() {

        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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


        out_zero = hardwareMap.get(TouchSensor.class, "out_zero");


        Pose startPose = new Pose(15, 40.0, Math.toRadians(0)); //TODO
        follower.setStartingPose(startPose);


        inisclosed = false;


        outisclosed = false;

        setIntakeState(0);
        setOuttakeState(0);
        setUpArmState(0);
        setUpArmState(0);



    }


    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        autonomousActionUpdate();


        previousgamepad2.copy(currentgamepad2);
        currentgamepad2.copy(gamepad2);
        previousgamepad1.copy(currentgamepad1);
        currentgamepad1.copy(gamepad1);
        if (gamepad2.triangle) {
            setOutArmState(1);
            setIntakeState(2);
            if (out.getCurrentPosition() == outarm_out_position) {
                setIntakeState(3);
                if (servo_intake.getPosition() == 1) {
                    setIntakeState(1);
                }
            }
        }
        if (gamepad2.circle) {
            setOutArmState(9);
            setUpArmState(9);
            setOuttakeState(9);
        }
        if (gamepad2.cross) {
            setIntakeState(10);
            setOuttakeState(10);
        }
        if (gamepad2.square) {
            setUpArmState(1);
            setOuttakeState(1);
        }
        if (gamepad2.right_bumper) {
            setOuttakeState(2);
        }


        telemetry.addData("gamepad2.rightstickx", gamepad2.right_stick_x);
        telemetry.addData("gamepad2.rightsticky", gamepad2.right_stick_y);
        telemetry.addData("up1 pos", up1.getCurrentPosition());
        telemetry.addData("up2 pos", up2.getCurrentPosition());
        telemetry.addData("servo intake pos", servo_intake.getPosition());
        telemetry.addData("servo rotate pos", servo_intake_rotate.getPosition());
        telemetry.addData("servo wrist ", servo_intake_wrist.getPosition());
        telemetry.addData("gamepad2 left stick x", gamepad2.left_stick_x);
        telemetry.addData("flip1", servo_outtake_flip1.getPosition());
        telemetry.addData("flip2", servo_outtake_flip2.getPosition());
        telemetry.addData("defend", defend);
        telemetry.addData("pos",follower.getPose());
        telemetry.addData("outisclosed", outisclosed);

        telemetry.update();
        follower.update();

    }
    //Adding left auto states here:
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

}
