package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Holonomic OpMode WORKING", group="Iterative Opmode")
//@Disabled
public class hDriveMain extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor  FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor;
    private DcMotorEx LiftMotor;
 //   private CRServo CMotor1, CMotor2, intakeServo;
    private int[] liftPos = {0, 1800, 3000, 4600};
    private int currentLiftPosition = 0;
    private Servo intakeServo, rotatorIntakeServo;
    HolonomicDrive holonomicDrive;
    boolean YIsPressed = false;
    boolean XIsPressed = false;
    boolean BIsPressed = false;
    boolean AIsPressed = false;
    boolean RBIsPressed = false;
    boolean LBIsPressed = false;
    boolean DPADLeftIsPressed = false;
    boolean DPADRightIsPressed = false;
    boolean LTIsPressed = false;
    boolean RTIsPressed = false;
    boolean BackIsPressed = false;
    boolean closed = false;
    boolean up = false;
    boolean forward = false;
    boolean B2IsPressed = false;
    boolean LB2IsPressed = false;
    boolean RB2IsPressed = false;
    boolean X2IsPressed = false;
    private boolean Y2IsPressed = false;
    private boolean A2IsPressed = false;
    boolean back2IsPressed = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontRightMotor  = hardwareMap.get(DcMotor.class, "frontRight");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        BackRightMotor  = hardwareMap.get(DcMotor.class, "backRight");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        LiftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        rotatorIntakeServo = hardwareMap.get(Servo.class, "rotatorServo");

        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        LiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LiftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if(forward) {
            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;
        } else if(!forward) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;
        }
        double RightJoyY2 = gamepad2.right_stick_y;

        boolean B_1Button = gamepad1.b;
        boolean A_1Button = gamepad1.a;
        boolean RB_1Button = gamepad1.right_bumper;
        boolean X_1Button = gamepad1.x;
        boolean Y_1Button = gamepad1.y;
        boolean LB_1Button = gamepad1.left_bumper;
        boolean DPL_1Button = gamepad1.dpad_left;
        boolean DPR_1Button = gamepad1.dpad_right;
        boolean backButton1 = gamepad1.back;

        boolean B_2Button = gamepad2.b;
        boolean A_2Button = gamepad2.a;
        boolean RB_2Button = gamepad2.right_bumper;
        boolean X_2Button = gamepad2.x;
        boolean Y_2Button = gamepad2.y;
        boolean LB_2Button = gamepad2.left_bumper;
        boolean backButton2 = gamepad1.back;

        // Open and close pick-point
        if(B_2Button && closed == false && B2IsPressed == false){
            closed = true;
            intakeServo.setPosition(0.0);
            B2IsPressed = true;
        }
        else if(B_2Button == false){
            B2IsPressed = false;
        }
        if(B_2Button && closed == true && B2IsPressed == false){
            closed = false;
            intakeServo.setPosition(0.25);
            B2IsPressed = true;
        }
        else if(B_2Button == false){
            B2IsPressed = false;
        }

        // sway arm
        if(X_2Button == true && X2IsPressed == false && currentLiftPosition >=1) {
            X2IsPressed = true;
            if(up) {
                up = false;
                rotatorIntakeServo.setPosition(0.655);
                forward = false;
            } else {
                up = true;
                telemetry.addLine("this is zero rn");
                telemetry.update();
                rotatorIntakeServo.setPosition(0.0);
                forward = true;
            }
        }
        else if(X_2Button == false) {
            X2IsPressed = false;
        }



        // Make the lift go UP
        if(Y_2Button && !Y2IsPressed && currentLiftPosition < (liftPos.length - 1)){
            A2IsPressed = true;
            currentLiftPosition += 1;
            LiftMotor.setTargetPosition(liftPos[currentLiftPosition]);
            LiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LiftMotor.setPower(0.8);
            Y2IsPressed = true;

        } else if(!Y_2Button) {
            Y2IsPressed = false;

        }

        //Make the lift go down
        if (A_2Button && !A2IsPressed && currentLiftPosition > 0){
            A2IsPressed = true;
            currentLiftPosition -= 1;
            LiftMotor.setTargetPosition(liftPos[currentLiftPosition]);
            LiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LiftMotor.setPower(-0.8);

        } else if(!A_2Button) {
            A2IsPressed = false;
        }

        //manual lift override
//        if(backButton2 && !back2IsPressed){
//            back2IsPressed = true;
//            currentLiftPosition = 0;
//            LiftMotor.setPower(RightJoyY2);
//            if(RightJoyY2 == 0.0) {
//                LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//        } else if(!backButton2) {
//            back2IsPressed = false;
//        }

        holonomicDrive.teleopDrive(x/0.75,y/0.75,z/0.75);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addLine("Current Lift position: " + currentLiftPosition + "\nCurrent Lift Ticks: " + liftPos[currentLiftPosition]);
        telemetry.addLine("Current Lift Motor Ticks: "  + LiftMotor.getCurrentPosition());
        telemetry.addLine("intake open: "  + LiftMotor.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}