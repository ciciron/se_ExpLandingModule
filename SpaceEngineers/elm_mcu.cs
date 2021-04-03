#region Prelude
using System;
using System.Linq;
using System.Text;
using System.Collections;
using System.Collections.Generic;

using VRageMath;
using VRage.Game;
using VRage.Collections;
using Sandbox.ModAPI.Ingame;
using VRage.Game.Components;
using VRage.Game.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using Sandbox.Game.EntityComponents;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage.Game.ObjectBuilders.Definitions;

// Change this namespace for each script you create.
namespace SpaceEngineers.LandingModule.MCU {
    public sealed class Program : MyGridProgram {
    // Your code goes between the next #endregion and #region
#endregion

List<IMyShipController> controllers;
List<IMyShipConnector> connectors;
List<IMyGasGenerator> generators;
List<IMyLandingGear> gears;
List<IMyThrust> thrusters;
List<IMyThrust> vertical_thrusters;
List<IMyGyro> gyros;

const double kDesiredVelocity = 100;

bool bStabilization = false;
bool bLaunch = false;
bool bLaunchOverride = false;
double VerticalThrustMax;

public Program() {
    //Controller(s)
	controllers = new List<IMyShipController>();
	GridTerminalSystem.GetBlocksOfType<IMyShipController>(controllers, b => b.IsSameConstructAs(Me));
	
	//Connector(s)
	connectors = new List<IMyShipConnector>();
	GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors, b => b.IsSameConstructAs(Me));
	
	//Gears
	gears = new List<IMyLandingGear>();
	GridTerminalSystem.GetBlocksOfType<IMyLandingGear>(gears, b => b.IsSameConstructAs(Me));
	
	//Gyroscope(s)
	gyros = new List<IMyGyro>();
	GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyros, b => b.IsSameConstructAs(Me));
	
	ResetGyroscope();
	
	generators = new List<IMyGasGenerator>();
	GridTerminalSystem.GetBlocksOfType<IMyGasGenerator>(generators, b => b.IsSameConstructAs(Me));
	
	//Thrusters
	VerticalThrustMax = 0;
	thrusters = new List<IMyThrust>();
	GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters, b => b.IsSameConstructAs(Me));
	if (controllers.Count > 0) {
		Matrix ctrl_mtx = new MatrixD();
		controllers[0].Orientation.GetMatrix(out ctrl_mtx);
		vertical_thrusters = new List<IMyThrust>();
		foreach (IMyThrust thruster in thrusters) {
			thruster.ThrustOverride = 0;
			Matrix mtx = new MatrixD();
			thruster.Orientation.GetMatrix(out mtx);
			if (mtx.Forward == ctrl_mtx.Down) {
				VerticalThrustMax += thruster.MaxEffectiveThrust;
				vertical_thrusters.Add(thruster);
			}
		}
	}
	bStabilization = false;
	bLaunch = false;

	Runtime.UpdateFrequency = UpdateFrequency.Update100 | UpdateFrequency.Update10;
}

public void Save()
{

}

public void Main(string args) {
    //Console & Antenna
	if ((updateSource & (UpdateType.Terminal | UpdateType.Trigger)) > 0) {
		Analyze(argument);
	}
	
	IMyShipController current_controller = GetController();
	Vector3D gravity = current_controller.GetNaturalGravity();
	
	//IGC
	if ((updateSource & UpdateType.IGC) == UpdateType.IGC) {
		ShowErrorOnLCD(current_controller as IMyTextSurfaceProvider, "IGC: " + argument);
	}
	
	if (argument.Length > 0)
		ShowErrorOnLCD(current_controller as IMyTextSurfaceProvider, argument + " - " + updateSource.ToString());
	
	if (gravity.Length() <= 0) {
		if (bStabilization) {
			bStabilization = false;
			ResetGyroscope();
		}
		if (bLaunch) {
			bLaunch = false;
			bLaunchOverride = false;
			ResetThrusters();
			Runtime.UpdateFrequency = UpdateFrequency.Update100 | UpdateFrequency.Update10;
			return;
		}
	}
	
	// tcs/10
	if ((updateSource & UpdateType.Update10) == UpdateType.Update10) {
		if (current_controller != null) {
			EquipmentControl(current_controller);
			if (!IsLanded() && !IsConnected()) {	
				if (bStabilization) {
					if (gyros.Count > 0) {
						IMyGyro gyroscope = gyros[0];
						ExecuteGyrostab(gyroscope, CalculateOrientation(current_controller, gravity));
					}
				}
			} else
				ResetGyroscope();
			UpdateDebugLCD(current_controller, Me as IMyTextSurfaceProvider);
		}
	}
	
	// tcs/100
	if ((updateSource & UpdateType.Update100) == UpdateType.Update100) {
		if (bLaunch) {
			bStabilization = true;
			if (!bLaunchOverride) {
				const double kThrustStarterAccel = 1.3;
				float mass = current_controller.CalculateShipMass().PhysicalMass;
				double thrust = mass * gravity.Length();
				thrust *= kThrustStarterAccel;
				foreach (IMyThrust thruster in vertical_thrusters) {
					thruster.ThrustOverride = Convert.ToSingle(thrust * thruster.MaxEffectiveThrust / VerticalThrustMax);
				}
				bLaunchOverride = true;
			} else
				Runtime.UpdateFrequency |= UpdateFrequency.Update1;
		}
	}
	
	// tcs/1
	if ((updateSource & UpdateType.Update1) == UpdateType.Update1) {
		
		if (bLaunchOverride) {	
			MyShipVelocities velocity = current_controller.GetShipVelocities();
			float mass = current_controller.CalculateShipMass().PhysicalMass;
			double thrust = mass * gravity.Length();
			double cur_vel = velocity.LinearVelocity.Length();
			if (cur_vel < kDesiredVelocity) {
				double k = Math.Cos(Math.PI * cur_vel / kDesiredVelocity / 2);
				thrust += (VerticalThrustMax - thrust) * k;
			} 
			foreach (IMyThrust thruster in vertical_thrusters) {
				thruster.ThrustOverride = Convert.ToSingle(thrust * thruster.MaxEffectiveThrust / VerticalThrustMax);
			}
		}
	}
}

internal void Analyze(string argument) {
	if (argument == "Stab") {
		bStabilization = true;
		return;
	}
	if (argument == "Unstab") {
		bStabilization = false;
		ResetGyroscope();
		return;
	}
	
	if (argument == "Launch") {
		Launch();
		return;
	}
	if (argument == "Landing") {
		bStabilization = true;
		Runtime.UpdateFrequency |= UpdateFrequency.Update1;
		return;
	}
	
	if (argument == "Stop") {
		Runtime.UpdateFrequency = UpdateFrequency.Update100 | UpdateFrequency.Update10;
		bStabilization = false;
		bLaunchOverride = false;
		bLaunch = false;
		ResetGyroscope();
		ResetThrusters();
		return;
	}
}

void Launch() {
	bLaunch = true;
	bLaunchOverride = false;
	
	foreach (IMyGasGenerator generator in generators) {
		generator.Enabled = true;
	}
	foreach (IMyLandingGear gear in gears) {
		gear.Unlock();
	}
	
}

bool IsLanded() {
	bool rv = false;
	foreach (IMyLandingGear gear in gears) {
		rv |= gear.IsLocked;
	}
	return rv;
}

bool IsConnected() {
	bool rv = false;
	foreach (IMyShipConnector connector in connectors) {
		rv |= (connector.Status == MyShipConnectorStatus.Connected);
	}
	return rv;
}

IMyShipController GetController() {
	foreach (IMyShipController ctrlr in controllers) {
		return ctrlr;
	}
	return null;
}

void ResetGyroscope() {
	foreach (IMyGyro device in gyros) {
		device.Yaw = 0;
		device.Pitch = 0;
		device.Roll = 0;
		device.GyroOverride = false;
	}
}

void ResetThrusters() {
	foreach (IMyThrust thruster in thrusters) {
		thruster.ThrustOverride = 0;
	}
}

void EquipmentControl(IMyShipController controller) {
	//Gyroscope & Thrusters
	bool control = controller.IsUnderControl;
	if (!control) {
		if (gears.Count == 0)
			return;	
		if (!IsLanded() && !IsConnected())
			return;
	}
	foreach (IMyGyro device in gyros) {
		device.Enabled = control;
	}
	foreach (IMyThrust device in thrusters) {
		device.Enabled = control;
	}
	
}


internal struct Orientation {
	private const double kThreshold = 0.01;
	public double Yaw;
	public double Pitch;
	public double Roll;
	public Orientation(double yaw, double pitch, double roll) {
		Yaw = yaw;
		Pitch = pitch;
		Roll = roll;
	}
	public bool NeedTurnYaw() {
		return Math.Abs(Yaw) >= kThreshold;
	}
	public bool NeedTurnPitch() {
		return Math.Abs(Pitch) >= kThreshold;
	}
	
	public bool NeedTurnRoll() {
		return Math.Abs(Roll) >= kThreshold;
	}
	
	public bool NeedCorrection() {
		return NeedTurnYaw() || NeedTurnPitch() || NeedTurnRoll();
	}
};

Orientation CalculateOrientation(IMyShipController ctrl, Vector3D gravity) {
	Vector3D down_vector = ctrl.WorldMatrix.Down;
	Vector3D fw_vector = ctrl.WorldMatrix.Forward;
	Vector3D left_vector = ctrl.WorldMatrix.Left;
	Vector3D right_vector = ctrl.WorldMatrix.Right;
	Vector3D gravity_norm = Vector3D.Normalize(gravity);
	
	double Pitch = gravity_norm.Dot(fw_vector);
    double Roll = gravity_norm.Dot(left_vector);
	
	return new Orientation(0, Pitch, Roll);
}

void ExecuteGyrostab(IMyGyro device, Orientation orient) {
	const double kPidPitchLinear = 0.2;
	const double kPidRollLinear = 0.05;
	const double kPidMinimal = 0.05;
	double yaw = orient.Yaw;
	double roll = 0;
	double pitch = 0;
	if (orient.NeedTurnPitch()) {
		double turn = Math.Max(kPidMinimal, kPidPitchLinear*Math.Abs(orient.Pitch));
		if (orient.Pitch > 0) 
		   pitch = -turn; 
		else
		   pitch = turn;
	}
	if (orient.NeedTurnRoll()) {
		double turn = Math.Max(kPidMinimal, kPidRollLinear*Math.Abs(orient.Roll));
		if (orient.Roll > 0) 
		   roll = turn; 
		else
		   roll = -turn;
	}
	device.Yaw = Convert.ToSingle(yaw);
	device.Pitch = Convert.ToSingle(pitch);
	device.Roll = Convert.ToSingle(roll);
	device.GyroOverride = true;
}

void UpdateDebugLCD(IMyShipController controller, IMyTextSurfaceProvider provider) {
	const byte kSurfaceNumber = 0;
	const Single kFontSize = 1.62f;
	
	IMyTextSurface surface = provider.GetSurface(kSurfaceNumber);
	surface.TextPadding = 1;
	surface.ContentType = ContentType.TEXT_AND_IMAGE;
	surface.FontSize = kFontSize;
	surface.BackgroundColor = Color.Blue;
	
	if (IsLanded()) {
		surface.WriteText("НА ЗЕМЛЕ\n");
		return;
	}
	if (IsConnected()) {
		surface.WriteText("ПРИСОЕДИНЕН\n");
		return;
	}
	
	var sb = new StringBuilder();
	sb.Append("X: " + controller.MoveIndicator.X.ToString()+"\n"); //Боковой
	sb.Append("Y: " + controller.MoveIndicator.Y.ToString()+"\n"); //Вертикальный
	sb.Append("Z: " + controller.MoveIndicator.Z.ToString()+"\n"); //Фронтальный
	
	sb.Append("RX: " + controller.RotationIndicator.X.ToString()+"\n"); //Боковой
	sb.Append("RY: " + controller.RotationIndicator.Y.ToString()+"\n"); //Вертикальный
	
	Vector3D gravity = controller.GetNaturalGravity();
	Vector3D gravityN = Vector3D.Normalize(gravity);
	double pitch = gravityN.Dot(controller.WorldMatrix.Forward);
    double roll = gravityN.Dot(controller.WorldMatrix.Left);
	
	sb.Append("Pitch: " + pitch.ToString("F3")+"\n");
	sb.Append("Roll: " + roll.ToString("F3")+"\n");
	
	sb.Append("VTM: " + VerticalThrustMax.ToString("F3")+"\n");
	
	sb.Append("Up: " + vertical_thrusters.Count.ToString()+"\n");
	
	
	surface.WriteText(sb);
	
}

void ShowErrorOnLCD(IMyTextSurfaceProvider provider, string error_string) {
	const byte kSurfaceNumber = 0;
	const Single kFontSize = 1.8f;
	IMyTextSurface surface = provider.GetSurface(kSurfaceNumber);
	surface.TextPadding = 1;
	surface.ContentType = ContentType.TEXT_AND_IMAGE;
	surface.FontSize = kFontSize;
	surface.BackgroundColor = Color.Blue;
	surface.WriteText(error_string);
}


#region PreludeFooter
    }
}
#endregion