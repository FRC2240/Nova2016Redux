#include <iostream>
#include "WPILib.h"
#include <CANTalon.h>
#include "PixyTracker.hpp"
#include <SmartDashboard/SmartDashboard.h>
#include "log.h"

#define _USE_MATH_DEFINES

const double PIXY_FIELD_OF_VIEW = 47.0;
const double PIXY_IMAGE_HEIGHT_PIXELS = 240.0;


class Robot: public IterativeRobot
{
private:
	/*--------------------------VARIABLES--------------------------*/
	float gatherSpeed 		= 0.0;
	float move 				= 0.0;
	float rotate			= 0.0;
	/*--------------------------BOOLEANS--------------------------*/
	bool shooterPosition 	= true; //false = out, true = in
	bool shooterWheels 		= false;
	bool wheelsGathererIn 	= false;
	bool wheelsGathererOut 	= false;
	bool done 				= false;
	bool shootingHigh 		= false;
	bool shootingLow 		= false;
	bool lastButton4        = false;
	bool lastButton1        = false;
	bool lastButton3        = false;

	//bool clamped 			= false;
	//bool clamping 			= false;
	/*--------------------------TIMERS----------------------------*/
	int gatherButtonTimer 	= 0;
	int shooterButtonTimer 	= 0;
	int clampButtonTimer 	= 0;
	int shooterTimer 		= 0;
	int clampTimer 			= 0;
	bool autonomousMode 	= false;
	//bool autoClampHigh		= false;
	//bool autoClampLow 		= false;
	//bool autoClampHighDone 	= false;
	//bool autoClampLowDone	= false;
	/*--------------------------OBJECTS---------------------------*/
	TalonSRX 	 *frontLeft;
	TalonSRX 	 *frontRight;
	TalonSRX 	 *backLeft;
	TalonSRX 	 *backRight;
	CANTalon 	 *lShooter;
	CANTalon 	 *rShooter;
	TalonSRX 	 *gatherer;
	TalonSRX 	 *shooterInOut;
	TalonSRX 	 *gathererWheels;
	TalonSRX     *elevator;
	Encoder 	 *lEnc;
	Encoder 	 *rEnc;
	RobotDrive 	 *drive;
	Joystick 	 *stick;
	//AnalogAccelerometer *ax;
	//AnalogAccelerometer *az;
	Servo        *cameraTilt;

	PixyTracker  *m_pixy;
	int m_signature = 1;
	PixyTracker::Target m_targets[2];

	PIDController *launchController = NULL;
	PIDController *gatherController = NULL;
	PIDController *turnController   = NULL;

	bool          testingLeft;
	bool          testingRight;

	// PID coefficients
	double      kLeftP, kLeftI, kLeftD, kLeftF, kLeftLowRPM, kLeftHighRPM;
	double      kRightP, kRightI, kRightD, kRightF, kRightLowRPM, kRightHighRPM;
	double      kGatherP, kGatherI, kGatherD, kGatherF;
	double		kLaunchP, kLaunchI, kLaunchD, kLaunchF;
	double      kTurnP, kTurnI, kTurnD, kTurnF;
	double		kLaunchMinAngle, kLaunchMaxAngle, kLaunchAngle;
	double		kCoeff0, kCoeff1;
	std::string kLogLevel;

	double	kGatherAngle;

	enum RobotState {
		kOperatorControl,
		kCentering,
		kAiming,
		kLaunching,
	} robotState = kOperatorControl;

	int stateTimer = 0;

	Preferences *prefs;


	// Output from the Turn PID Controller
	class TurnPIDOutput : public PIDOutput {
	public:
		double correction = 0.0;
		void PIDWrite(double output) {
			correction = output;
		}
	} turnPIDOutput;

	// Data source for the Turn PID Controller
	class TurnPIDSource : public PIDSource {
	public:
		TurnPIDSource(PixyTracker *pixy, int signature, PixyTracker::Target *targets)
	    : m_isAcquired(false),
		  m_pixy(pixy),
		  m_signature(signature),
		  m_targets(targets) {}
		double PIDGet() {
			int count = m_pixy->getBlocksForSignature(m_signature, 1, m_targets);
			if (count == 1) {
				//std::cout << "x: " << m_targets[0].block.x << " y: " << m_targets[0].block.y
				//		  << " h: " << m_targets[0].block.height << " w: " << m_targets[0].block.width <<std::endl;
				m_isAcquired = true;
				return (160.0 - m_targets[0].block.x);
			}
			m_isAcquired = false;
			return 0;
		}
		bool acquired() {
			return m_isAcquired;
		}
		void reset() {
			m_isAcquired = false;
		}
	private:
		bool m_isAcquired;
		PixyTracker *m_pixy;
		int m_signature;
		PixyTracker::Target *m_targets;
	};

	TurnPIDSource *turnPIDSource;


	// Output from the Gather PID Controller
	class GatherPIDOutput : public PIDOutput {
	public:
		double correction = 0.0;
		void PIDWrite(double output) {
			correction = output;
		}
	} gatherPIDOutput;

	// Data source for the Gather PID Controller
	class GatherPIDSource : public PIDSource {
	public:
		GatherPIDSource() {
			ax = new AnalogAccelerometer(2);
			az = new AnalogAccelerometer(3);
		}
		double PIDGet() {
			double x = ax->GetAcceleration();
			double z = az->GetAcceleration();
			double g = sqrt(x*x+z*z);
			return (90.0 + (180.0/M_PI)*asin(x/g));
		}
	private:
		AnalogAccelerometer *ax;
		AnalogAccelerometer *az;
	} gatherPIDSource;

	// Output from the Launch PID Controller
	class LaunchPIDOutput : public PIDOutput {
	public:
		double correction = 0.0;
		void PIDWrite(double output) {
			correction = output;
		}
	} launchPIDOutput;

	// Data source for the Launch PID Controller
	class LaunchPIDSource : public PIDSource {
	public:
		LaunchPIDSource() {
			ax = new AnalogAccelerometer(0);
			az = new AnalogAccelerometer(1);
		}
		double PIDGet() {
			double x = ax->GetAcceleration();
			double z = az->GetAcceleration();
			double g = sqrt(x*x+z*z);
			return (90.0 + (180.0/M_PI)*asin(x/g));
		}
	private:
		AnalogAccelerometer *ax;
		AnalogAccelerometer *az;
	} launchPIDSource;

	// Read data from the Preferences Panel
	void getPreferences()
	{
		kLeftP = prefs->GetDouble("kLeftP", 0.025);
		kLeftI = prefs->GetDouble("kLeftI", 0.0);
		kLeftD = prefs->GetDouble("kLeftD", 0.2);
		kLeftF = prefs->GetDouble("kLeftF", 0.03);
		kLeftLowRPM = prefs->GetDouble("kLeftLowRPM", -1000.0);
		kLeftHighRPM = prefs->GetDouble("kLeftHighRPM", -2400.0);

		kRightP = prefs->GetDouble("kRightP", 0.025);
		kRightI = prefs->GetDouble("kRightI", 0.0);
		kRightD = prefs->GetDouble("kRightD", 0.2);
		kRightF = prefs->GetDouble("kRightF", 0.03);
		kRightLowRPM = prefs->GetDouble("kRightLowRPM", 1000.0);
		kRightHighRPM = prefs->GetDouble("kRightHighRPM", 2400.0);

		kLaunchP = prefs->GetDouble("kLaunchP", 0.2);
		kLaunchI = prefs->GetDouble("kLaunchI", 0.004);
		kLaunchD = prefs->GetDouble("kLaunchD", 0.5);
		kLaunchF = prefs->GetDouble("kLaunchF", 0.0);

		kGatherP = prefs->GetDouble("kGatherP", 0.05);
		kGatherI = prefs->GetDouble("kGatherI", 0.0);
		kGatherD = prefs->GetDouble("kGatherD", 0.1);
		kGatherF = prefs->GetDouble("kGatherF", 0.0);

		kTurnP = prefs->GetDouble("kTurnP", 0.025);
		kTurnI = prefs->GetDouble("kTurnI", 0.004);
		kTurnD = prefs->GetDouble("kTurnD", 0.06);
		kTurnF = prefs->GetDouble("kTurnF", 0.0);

		kGatherAngle = prefs->GetDouble("kGatherAngle", 113.0);

		kLaunchMinAngle = prefs->GetDouble("kLaunchMinAngle", 23.5);
		kLaunchMaxAngle = prefs->GetDouble("kLaunchMaxAngle", 36.7);
		kLaunchAngle = prefs->GetDouble("kLaunchAngle", 30.0);

		kCoeff0 = prefs->GetDouble("kCoeff0", 0.144);
		kCoeff1 = prefs->GetDouble("kCoeff1", 19.8);

		kLogLevel = prefs->GetString("kLogLevel", "INFO");

		Log::SetLevel(Log::FromString(kLogLevel));

		LOGGER(INFO) << "[getPreferences] LeftRPM:" << kLeftHighRPM << " RightRPM: " << kRightHighRPM;
	}

	double calcLaunchAngle(double y) {
		return kCoeff0*y + kCoeff1;
	}

	void stateOperatorControl() {
		// DRIVING
		move = stick->GetRawAxis(1) * -1.0;
		rotate = stick->GetRawAxis(4) * -1.0;

		// Deadband
		if (fabs(move) < 0.1) {
			move = 0.0;
		}
		if (fabs(rotate) < 0.15) {
			rotate = 0.0;
		}
		drive->ArcadeDrive(move, rotate, false);

		// Joystick Buttons
		bool button4 = stick->GetRawButton(4);
		bool button1 = stick->GetRawButton(1);
		bool button5 = stick->GetRawButton(5);
		bool button6 = stick->GetRawButton(6);
		bool button3 = stick->GetRawButton(3);

		// Manual Gatherer
		if (stick->GetRawAxis(2) != 0) {
			gatherSpeed = stick->GetRawAxis(2);
			LOGGER(DEBUG) << "[stateOperatorControl] Gather Angle:" << gatherPIDSource.PIDGet();

		}
		else if (stick->GetRawAxis(3) != 0) {
			gatherSpeed = stick->GetRawAxis(3) * -1;
			LOGGER(DEBUG) << "[stateOperatorControl] Gather Angle:" << gatherPIDSource.PIDGet();
		}
		else {
			gatherSpeed = 0.0;
		}
		gatherer->Set(gatherSpeed);

		// Launch Angle
		double launcherAngle = launchPIDSource.PIDGet();
		if (button5 && !button6  && (launcherAngle < kLaunchMaxAngle)) {
			elevator->Set(-0.5); // Up
			LOGGER(DEBUG) << "[stateOperatorControl] Launcher Angle:" << launcherAngle;
		} else if (button6 && !button5 && (launcherAngle > kLaunchMinAngle)) {
			LOGGER(DEBUG) << "[stateOperatorControl] Launcher Angle:" << launcherAngle;
			elevator->Set(0.5); // Down
		} else {
			elevator->Set(0.0);
		}

		// Auto-Gather
		if (button3 && !lastButton3) {
			wheelsGathererIn = !wheelsGathererIn;
			gatherController->SetSetpoint(kGatherAngle);
			gatherController->Enable();
		}
		if (wheelsGathererIn) {
			gathererWheels->Set(1.0);
			gatherer->Set(gatherPIDOutput.correction);
			LOGGER(DEBUG) << "[stateOperatorControl] Gather Correction:" << gatherPIDOutput.correction
					      << " Gather Angle: " << gatherPIDSource.PIDGet();
		} else {
			gathererWheels->Set(0.0);
			gatherController->Disable();
		}

		if (button4 && !lastButton4) {
			stateTimer   = 0;
			robotState   = kCentering;
			shootingHigh = true;
		}
		if (button1 && !lastButton1) {
			stateTimer   = 0;
			robotState   = kLaunching;
			shootingHigh = true;
		}
		lastButton4 = button4;
		lastButton1 = button1;
		lastButton3 = button3;
	}

	void stateCentering() {
		stateTimer++;

		if (stateTimer == 1) {
			turnController->SetSetpoint(0.0);
			turnController->Enable();
			return;
		} else if (stateTimer < 10) {
			return;
	    } else if (stateTimer < 120) {
			if (!turnPIDSource->acquired()) {
				robotState = kOperatorControl;
				turnController->Disable();
				turnPIDSource->reset();
				LOGGER(ERROR) << "[stateCentering] No Target Found";
				return;
			}
			drive->ArcadeDrive(0.0, -turnPIDOutput.correction, false);
			LOGGER(DEBUG) << "[stateCentering] Timer: " << stateTimer << " SetPoint: " << turnController->GetSetpoint()
					      << " Offset: " << turnPIDSource->PIDGet() << " Correction: " << turnPIDOutput.correction;

			if ((fabs(turnPIDOutput.correction) < 0.10) && (fabs(turnPIDSource->PIDGet()) < 3)) {
				drive->ArcadeDrive(0.0, 0.0, false);
				turnController->Disable();
				robotState = kAiming;
				stateTimer = 0;
				turnPIDSource->reset();
			}
		} else {
			turnController->Disable();
			turnPIDSource->reset();
			drive->ArcadeDrive(0.0, 0.0, false);
			robotState = kOperatorControl;
			LOGGER(ERROR) << "[stateCentering] FAILED, Timer: " << stateTimer << " SetPoint: " << turnController->GetSetpoint()
				          << " Offset: " << turnPIDSource->PIDGet() << " Correction: " << turnPIDOutput.correction;
		}
	}

	void stateAiming() {
		stateTimer++;
		if (stateTimer == 1) {
			// calculate launcher angle
			double angle = calcLaunchAngle(m_targets[0].block.y);
			LOGGER(INFO) << "[stateAiming] Setting Launch Angle: " << angle;

			if (angle > 46.0) {
				LOGGER(ERROR) << "[stateAiming] Target is out of range";
				robotState = kOperatorControl;
				return;
			}

			launchController->SetSetpoint(angle);
			launchController->Enable();
			return;
		} else if (stateTimer < 5) {
			return;
		} else if (stateTimer < 150) {
			LOGGER(DEBUG) << "[stateAiming] Timer: " << stateTimer << " SetPoint: " << launchController->GetSetpoint()
					      << " Angle: " << launchPIDSource.PIDGet() << " Correction: " << launchPIDOutput.correction;
			elevator->Set(-launchPIDOutput.correction);

			if ((fabs(launchPIDOutput.correction) < 0.2) &&
				(fabs(launchPIDSource.PIDGet()-launchController->GetSetpoint()) < 0.25)) {
				elevator->Set(0.0);
				launchController->Disable();
				robotState = kLaunching;
				stateTimer = 0;

				LOGGER(DEBUG) << "[stateAiming] Target x: " << m_targets[0].block.x << " y: " << m_targets[0].block.y
						      << " h: " << m_targets[0].block.height << " w: " << m_targets[0].block.width;
			}
		} else {
			launchController->Disable();
			elevator->Set(0.0);
			robotState = kOperatorControl;
			LOGGER(ERROR) << "[stateAiming] Aiming Failed, Timer: " << stateTimer << " Correction: "
					      << launchPIDOutput.correction;
		}
	}

	void stateLaunching() {
		stateTimer++;
		if (stateTimer == 1) {
			if (shootingHigh) {
				rShooter->Set(kRightHighRPM);
				lShooter->Set(kLeftHighRPM);
			} else {
				rShooter->Set(kRightLowRPM);
				lShooter->Set(kLeftLowRPM);
			}
		} else if (stateTimer > 50 && stateTimer <= 65) {
			shooterInOut->Set(-1.0);
			LOGGER(DEBUG) << "[stateLaunching] Timer: " << stateTimer << " Angle: " << launchPIDSource.PIDGet()
					      << " Right RPM: " << rShooter->GetSpeed() << " Left RPM: " << lShooter->GetSpeed();
		} else if (stateTimer > 65 && stateTimer <= 70) {
			shooterInOut->Set(0.0);
		} else if (stateTimer > 70 && stateTimer <= 75) {
			shooterInOut->Set(0.6);
		} else if (stateTimer > 75) {
			shootingHigh = false;
			stateTimer = 0;
			rShooter->Set(0.0);
			lShooter->Set(0.0);
			shooterInOut->Set(0.0);
			robotState = kOperatorControl;
		}
	}

	void resetPIDControllers() {
		delete launchController;
		delete gatherController;
		delete turnController;

		launchController = new PIDController(kLaunchP, kLaunchI, kLaunchD, kLaunchF, &launchPIDSource, &launchPIDOutput);
		launchController->SetInputRange(kLaunchMinAngle, kLaunchMaxAngle);
		launchController->SetOutputRange(-1.0, 1.0);
		launchController->SetAbsoluteTolerance(0.05);
		launchController->SetContinuous(false);

		gatherController = new PIDController(kGatherP, kGatherI, kGatherD, kGatherF, &gatherPIDSource, &gatherPIDOutput);
		gatherController->SetInputRange(0.0, 180.0);
		gatherController->SetOutputRange(-0.5, 0.5);
		gatherController->SetAbsoluteTolerance(0.1);
		gatherController->SetContinuous(false);

		turnController = new PIDController(kTurnP, kTurnI, kTurnD, kTurnF, turnPIDSource, &turnPIDOutput);
		turnController->SetInputRange(-160.0, 160.0);
		turnController->SetOutputRange(-0.4, 0.4);
		turnController->SetAbsoluteTolerance(0.05);
		turnController->SetContinuous(false);

	}

	void RobotInit()
	{
		prefs = Preferences::GetInstance();

		stick 		 	= new Joystick(0);
		frontLeft 	 	= new TalonSRX(1);

		backLeft 	 	= new TalonSRX(0);
		frontRight 	 	= new TalonSRX(8);
		backRight 	 	= new TalonSRX(9);
		shooterInOut 	= new TalonSRX(6);
		gatherer 	   	= new TalonSRX(5); // Gatherer up/down
		gathererWheels 	= new TalonSRX(4); // Gatherer in and out wheels
		elevator        = new TalonSRX(7); // Actuator for shooter
		lShooter 		= new CANTalon(1); // Left shooter wheels
		rShooter 	   	= new CANTalon(0); // Right shooter wheels

		rShooter->SetTalonControlMode(CANTalon::kSpeedMode);
		rShooter->SetSensorDirection(false);
		rShooter->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		rShooter->ConfigNominalOutputVoltage(0.0, 0.0);
		rShooter->ConfigPeakOutputVoltage(12.0, -12.0);

		lShooter->SetTalonControlMode(CANTalon::kSpeedMode);
		lShooter->SetSensorDirection(false);
		lShooter->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		lShooter->ConfigNominalOutputVoltage(0.0, 0.0);
		lShooter->ConfigPeakOutputVoltage(12.0, -12.0);

		cameraTilt = new Servo(3);

		drive = new RobotDrive(backLeft, frontLeft, backRight, frontRight);
		drive->SetSafetyEnabled(false);

		m_pixy = new PixyTracker();
		m_pixy->startVideo();

		turnPIDSource = new TurnPIDSource(m_pixy, m_signature, m_targets);
	}

	void AutonomousInit()
	{
		getPreferences();
		done = false;
	}

	void AutonomousPeriodic()
	{
		/*if(!done && !autonomousMode)
		{
			Auto("Move", -.2, 100, 1.0, .975);
			Auto("Move", -0.6, 600, 1.0, .975);
			Auto("Move", .2, 100, .85, 1.0);
			Auto("Move", 0.6, 600, .85, 1.0);
			Auto("Move", -.2, 100, 1.0, .975);
			Auto("Move", -0.6, 600, 1.0, .975);
			done = true;
		}*/
		if(!done/* && autonomousMode*/)
		{
			Auto("Move", -.2, 100, 1.0, 1.0);
			Auto("Move", -.6, 725, 1.0, 1.0);
			done = true;
		}
		else
		{
			frontLeft->Set(0.0);
			backLeft->Set(0.0);
			frontRight->Set(0.0);
			backRight->Set(0.0);
		}
	}

	void TeleopInit()
	{
		getPreferences();

		rShooter->SetF(kRightF);
		rShooter->SetP(kRightP);
		rShooter->SetI(kRightI);
		rShooter->SetD(kRightD);

		lShooter->SetF(kLeftF);
		lShooter->SetP(kLeftP);
		lShooter->SetI(kLeftI);
		lShooter->SetD(kLeftD);

		resetPIDControllers();
	}

	void TeleopPeriodic()
	{
		cameraTilt->Set(0.3);

		switch (robotState) {
		case kCentering:
			stateCentering();
			break;
		case kAiming:
			stateAiming();
			break;
		case kLaunching:
			stateLaunching();
			break;
		default:
		case kOperatorControl:
			stateOperatorControl();
			break;
		}
	}
	void Auto(std::string command, float power, float time, float correctL, float correctR)
	{
		if(command == "Move")
		{
			while(time >= 0)
			{
				if(power < 0)
				{
					correctR = 1.0;
				}
				//drive->ArcadeDrive(power, 0, false);
				frontLeft->Set(-power * correctL);
				backLeft->Set(-power * correctL);
				frontRight->Set(power * correctR);
				backRight->Set(power * correctR);
				Wait(0.005);
				time--;
			}
		}
		else if(command == "Turn")
		{
			while(time >= 0)
			{
				//drive->ArcadeDrive(0, power, false);
				Wait(0.005);
				time--;
			}
		}
	}

	void TestInit()
	{
		getPreferences();
		testingLeft  = false;
		testingRight = false;

		rShooter->SetF(kRightF);
		rShooter->SetP(kRightP);
		rShooter->SetI(kRightI);
		rShooter->SetD(kRightD);

		lShooter->SetF(kLeftF);
		lShooter->SetP(kLeftP);
		lShooter->SetI(kLeftI);
		lShooter->SetD(kLeftD);
	}

	void TestPeriodic()
	{
		bool nowButton4 = stick->GetRawButton(4);
		bool nowButton1 = stick->GetRawButton(1);

		if (nowButton4 && !lastButton4) {
			testingLeft = !testingLeft;
		}
		if (nowButton1 && !lastButton1) {
			testingRight = !testingRight;
		}
		lastButton4 = nowButton4;
		lastButton1 = nowButton1;

		if (testingRight) {
			rShooter->Set(kRightHighRPM);
			std::cout << rShooter->GetSpeed() << std::endl;
		} else {
			rShooter->Set(0.0);
		}
		if (testingLeft) {
			lShooter->Set(kLeftHighRPM);
			std::cout << lShooter->GetSpeed() << std::endl;
		} else {
			lShooter->Set(0.0);
		}
	}
};

START_ROBOT_CLASS(Robot);
