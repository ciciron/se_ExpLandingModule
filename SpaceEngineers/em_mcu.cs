#region Prelude
using System;
using System.Linq;
using System.Text;
using System.Collections;
using System.Collections.Generic;

using VRageMath;
using VRage.Game;
using VRage.Collections;
using VRage.Game.GUI.TextPanel;
using VRage.Game.Components;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ObjectBuilders.Definitions;

using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using Sandbox.Game.EntityComponents;
using SpaceEngineers.Game.ModAPI.Ingame;

// Change this namespace for each script you create.
namespace ExplorersModule.MCU
{
    public sealed class Program : MyGridProgram
    {
        // Your code goes between the next #endregion and #region
        #endregion

        PeripheryControl periphery;
        FuelControl gas_system;
        FlyControl fly;

        const double kDesireVelocity = 100;
        bool previous_control_state = false;

        public Program()
        {
            List<IMyCockpit> controllers = new List<IMyCockpit>();
            GridTerminalSystem.GetBlocksOfType<IMyCockpit>(controllers, b => b.IsSameConstructAs(Me) && b.IsMainCockpit);

            if (controllers.Count == 0)
            {
                Echo("No main controller found");
                ShowErrorOnLCD(Me as IMyTextSurfaceProvider, "No\n Controller");
                return;
            }
            var controller = controllers[0];
            periphery = new PeripheryControl(GridTerminalSystem, Me, controller);
            gas_system = new FuelControl(GridTerminalSystem, Me);
            fly = new FlyControl(GridTerminalSystem, Me, controller);

            Runtime.UpdateFrequency = UpdateFrequency.Update100;
            Echo("Init OK");
        }

        public void Save()
        {

        }
 
        void Idle(IMyShipController controller)
        {
            if (periphery.IsConnected() || periphery.IsLanded())
            {
                if (!controller.IsUnderControl)
                {
                    previous_control_state = false;
                    fly.TurnGyroscope(false);
                    Runtime.UpdateFrequency = UpdateFrequency.Update100;
                }
            }
            if (controller.IsUnderControl)
            {
                if (!previous_control_state)
                {
                    fly.Online();
                    previous_control_state = true;
                }
                Runtime.UpdateFrequency |= UpdateFrequency.Update100 | UpdateFrequency.Update10;
            }
            fly.Idle();
        }

        public void Main(string argument, UpdateType updateSource)
        {
            IMyShipController controller = fly.Controller();

            //Console & Triggers
            if ((updateSource & (UpdateType.Terminal | UpdateType.Script | UpdateType.Trigger)) > 0)
            {
                Analyze(argument);
            }

            //IGC
            if ((updateSource & UpdateType.IGC) > 0)
            {
                Echo("IGC: " + argument);
            }

            //TCS
            if ((updateSource & (UpdateType.Update1 | UpdateType.Update10 | UpdateType.Update100)) > 0)
            {
                if ((updateSource & UpdateType.Update100) > 0)
                {
                    Idle(controller);
                }
                if ((updateSource & UpdateType.Update10) > 0)
                {
                    fly.Tick();
                }
            }
        }

        void Analyze(string argument)
        {
            if (argument == "Launch")
            {
                gas_system.Start();
                periphery.Undock();
                fly.Launch();
                return;
            }
            if (argument == "Stop")
            {
                fly.Stop();
                return;
            }
        }

        void ShowErrorOnLCD(IMyTextSurfaceProvider provider, string error_string)
        {
            const int kSurfaceNumber = 0;
            const Single kFontSize = 1.8f;
            IMyTextSurface surface = provider.GetSurface(kSurfaceNumber);
            surface.TextPadding = 1;
            surface.ContentType = ContentType.TEXT_AND_IMAGE;
            surface.FontSize = kFontSize;
            surface.BackgroundColor = Color.Blue;
            surface.WriteText(error_string);
        }

        //V0.1
        internal sealed class FlyControl
        {
            public enum Mode { Classic, Hover};
            public FlyControl(IMyGridTerminalSystem grid_sys, IMyTerminalBlock source, IMyShipController ctrl, Mode m = Mode.Classic)
            {
                controller = ctrl;
                mode = m;
                gyros = new List<IMyGyro>();
                grid_sys.GetBlocksOfType<IMyGyro>(gyros, b => b.IsSameConstructAs(source));
                ResetGyroscope();

                List<IMyThrust> thrusters = new List<IMyThrust>();
                thr_fw = new List<IMyThrust>();
                thr_up = new List<IMyThrust>();
                thr_down = new List<IMyThrust>();
                grid_sys.GetBlocksOfType<IMyThrust>(thrusters, b => b.IsSameConstructAs(source));

                thrust_max_countdown = 0;
                launch_active = false;
                maxA = 0;
                maxL = 0;

                double force_fw = 0;
                double force_up = 0;
                Matrix ctrl_mtx = new MatrixD();
                controller.Orientation.GetMatrix(out ctrl_mtx);
                foreach (IMyThrust thruster in thrusters)
                {
                    thruster.ThrustOverride = 0;
                    Matrix mtx = new MatrixD();
                    thruster.Orientation.GetMatrix(out mtx);
                    if (mtx.Forward == ctrl_mtx.Backward)
                    {
                        force_fw += thruster.MaxEffectiveThrust;
                        thr_fw.Add(thruster);
                    }
                    if (mtx.Forward == ctrl_mtx.Down)
                    {
                        force_up += thruster.MaxEffectiveThrust;
                        thr_up.Add(thruster);
                    }
                    if (mtx.Forward == ctrl_mtx.Up)
                    {
                        thr_down.Add(thruster);
                        if (controller.GetNaturalGravity().Length() >= kGravityThreshold)
                            thruster.Enabled = false;
                    }
                }
                thrust_force = new Vector2D(force_fw, force_up);
            }

            public IMyShipController Controller()
            {
                return controller;
            }

            public void Launch()
            {
                thrust_max_countdown = 3;
                launch_active = true;
            }

            public void Stop()
            {
                launch_active = false;
                ResetGyroscope();
                ResetThrusters();
            }

            public void ResetGyroscope()
            {
                foreach (IMyGyro device in gyros)
                {
                    device.Yaw = 0;
                    device.Pitch = 0;
                    device.Roll = 0;
                    device.GyroOverride = false;
                }
            }

            void ResetThrusters()
            {
                foreach (IMyThrust thruster in thr_fw)
                    thruster.ThrustOverride = 0;
                foreach (IMyThrust thruster in thr_up)
                    thruster.ThrustOverride = 0;
            }

            public void TurnGyroscope(bool state)
            {
                foreach (IMyGyro device in gyros)
                    device.Enabled = state;
            }

            public void ExecuteHorizontalStabilization()
            {
                var gravity = controller.GetNaturalGravity();
                var orientation = CalculateHorizontalOrientation(gravity);
                foreach (IMyGyro gyroscope in gyros)
                    ExecuteGyroscope(gyroscope, orientation);
            }

            void ExecuteGyroscope(IMyGyro device, Orientation orient)
            {
                const double kPidPitchLinear = 0.2;
                const double kPidRollLinear = 0.05;
                const double kPidMinimal = 0.01;
                double yaw =  orient.Yaw;
                double roll = 0;
                double pitch = 0;
                if (orient.NeedTurnPitch())
                {
                    double turn = Math.Max(kPidMinimal, kPidPitchLinear * Math.Abs(orient.Pitch));
                    if (orient.Pitch > 0)
                        pitch = -turn;
                    else
                        pitch = turn;
                }
                if (orient.NeedTurnRoll())
                {
                    double turn = Math.Max(kPidMinimal, kPidRollLinear * Math.Abs(orient.Roll));
                    if (orient.Roll > 0)
                        roll = turn;
                    else
                        roll = -turn;
                }
                Vector3D axis = controller.WorldMatrix.Up * yaw +
                            controller.WorldMatrix.Right * pitch +
                            controller.WorldMatrix.Backward * roll;
                device.Yaw = Convert.ToSingle(axis.Dot(device.WorldMatrix.Up));
                device.Pitch = Convert.ToSingle(axis.Dot(device.WorldMatrix.Right));
                device.Roll = Convert.ToSingle(axis.Dot(device.WorldMatrix.Backward));
                device.GyroOverride = orient.NeedTurnYaw() | orient.NeedTurnPitch() | orient.NeedTurnRoll();
            }

            public void Idle()
            {
                Vector3D gravity = controller.GetNaturalGravity();
                float mass = controller.CalculateShipMass().PhysicalMass;
                bool gravity_enabled = gravity.Length() >= kGravityThreshold;
                if (gravity_enabled)
                {
                    double thr_frc_fw = 0;
                    double thr_frc_up = 0;
                    foreach (IMyThrust thruster in thr_fw)
                        thr_frc_fw += thruster.MaxEffectiveThrust;
                    foreach (IMyThrust thruster in thr_up)
                        thr_frc_up += thruster.MaxEffectiveThrust;
                    thrust_force = new Vector2D(thr_frc_fw, thr_frc_up);
                }
                maxA = Convert.ToSingle(kThrottle * thrust_force.Y / mass - gravity.Length());
                maxL = Convert.ToSingle(kDesireVelocity * Convert.ToSingle(kDesireVelocity / maxA) - maxA * Math.Pow(Convert.ToSingle(kDesireVelocity / maxA), 2) / 2);
                foreach (IMyThrust thruster in thr_down)
                    thruster.Enabled = !gravity_enabled;
                if (thrust_max_countdown > 0)
                    thrust_max_countdown--;
            }

            public void Tick()
            {
                Vector3D gravity = controller.GetNaturalGravity();
                if (gravity.Length() <= kGravityThreshold)
                {
                    if (launch_active)
                        Stop();
                    return;
                }
                
                MyShipVelocities velocity = controller.GetShipVelocities();
                float mass = controller.CalculateShipMass().PhysicalMass;
                if (launch_active)
                {
                    const double kThrustStarterAccel = 1.25;
                    if (controller.MoveIndicator.Length() > 0)
                    {
                        Stop();
                        return;
                    }
                    double gravity_force = mass * gravity.Length();
                    double thrust = gravity_force;
                    if (thrust_max_countdown == 0)
                    {
                        double cur_vel = velocity.LinearVelocity.Length();
                        double k = Math.Cos(Math.PI * cur_vel / kDesireVelocity / 2);
                        thrust += (thrust_force.Y - gravity_force) * k;
                    }
                    else
                        thrust *= kThrustStarterAccel;
                    foreach (IMyThrust thruster in thr_up)
                        thruster.ThrustOverride = Convert.ToSingle(thrust * thruster.MaxEffectiveThrust / thrust_force.Y);
                    ExecuteHorizontalStabilization();
                    return;
                }
                if (gravity.Length() > 0)
                {
                    if (controller.MoveIndicator.Y == 0)
                    {
                        ResetGyroscope();
                        foreach (IMyThrust thruster in thr_up)
                            thruster.ThrustOverride = 0;
                        return;
                    }
                    //Landing
                    if ((controller.MoveIndicator.Y < 0) && controller.DampenersOverride)
                    {
                        const double kMinimalVelocity = 3;
                        double kHyroDeviation = 2 / gyros.Count;
                        if (velocity.LinearVelocity.Length() <= kMinimalVelocity)
                        {
                            ResetGyroscope();
                            ResetThrusters();
                            return;
                        }
                        double elevation = 0;
                        Vector3D linvel = Vector3D.Transform(velocity.LinearVelocity, Matrix.Invert(Matrix.Normalize(controller.CubeGrid.WorldMatrix)).GetOrientation());
                        if (controller.TryGetPlanetElevation(MyPlanetElevation.Surface, out elevation))
                        {
                            const double kForSideSpeedDump = 0.25;
                            var orientation = CalculateHorizontalOrientation(gravity);
                            orientation.Pitch -= kHyroDeviation * linvel.Z / kDesireVelocity;
                            orientation.Roll -= kHyroDeviation * linvel.X / kDesireVelocity;
                            foreach (IMyGyro gyroscope in gyros)
                                ExecuteGyroscope(gyroscope, orientation);
                            double thrust = mass * gravity.Length();
                            if (maxL > elevation)
                            {
                                const double kSafeElevation = 75;
                                const double kAverageSpeed = 25;
                                const double kSafeSpeed = 5;
                                double max_speed = elevation > kSafeElevation ? kAverageSpeed : kSafeSpeed;
                                double cur_down = velocity.LinearVelocity.Dot(Vector3D.Normalize(gravity));
                                if (cur_down > max_speed)
                                    thrust += (thrust_force.Y - thrust) * Math.Sin(Math.PI / 2 * cur_down / kDesireVelocity);
                                else if (cur_down < max_speed)
                                    thrust *= kThrottle;
                            }
                            else
                                thrust *= kForSideSpeedDump;
                            foreach (IMyThrust thruster in thr_up)
                                    thruster.ThrustOverride = Convert.ToSingle(thrust * thruster.MaxEffectiveThrust / thrust_force.Y);
                        }
                        return;
                    }
                }
                //Hover
                if (mode == Mode.Hover)
                {
                    double kHoverDeviation = 0.75 / gyros.Count;
                    var orientation = CalculateHorizontalOrientation(gravity);
                    orientation.Pitch += kHoverDeviation * controller.MoveIndicator.Z;
                    orientation.Roll += kHoverDeviation * controller.MoveIndicator.X;
                    foreach (IMyGyro gyroscope in gyros)
                        ExecuteGyroscope(gyroscope, orientation);
                }
            }

            public void Online()
            {
                TurnGyroscope(true);
                ResetGyroscope();
            }

            struct Orientation
            {
                const double kThreshold = 0.01;
                public double Yaw;
                public double Pitch;
                public double Roll;
                public Orientation(double yaw, double pitch, double roll)
                {
                    Yaw = yaw;
                    Pitch = pitch;
                    Roll = roll;
                }
                public bool NeedTurnYaw()
                {
                    return Math.Abs(Yaw) >= kThreshold;
                }
                public bool NeedTurnPitch()
                {
                    return Math.Abs(Pitch) >= kThreshold;
                }
                public bool NeedTurnRoll()
                {
                    return Math.Abs(Roll) >= kThreshold;
                }
                public bool NeedCorrection()
                {
                    return NeedTurnYaw() || NeedTurnPitch() || NeedTurnRoll();
                }
            };

            Orientation CalculateHorizontalOrientation(Vector3D gravity)
            {
                Vector3D fw_vector = controller.WorldMatrix.Forward;
                Vector3D left_vector = controller.WorldMatrix.Left;
                Vector3D gravity_norm = Vector3D.Normalize(gravity);

                double Pitch = gravity_norm.Dot(fw_vector);
                double Roll = gravity_norm.Dot(left_vector);

                return new Orientation(0, Pitch, Roll);
            }

            IMyShipController controller;
            Mode mode;
            List<IMyGyro> gyros;
            List<IMyThrust> thr_fw;
            List<IMyThrust> thr_up;
            List<IMyThrust> thr_down;
            Vector2D thrust_force;

            int thrust_max_countdown;
            bool launch_active;
            float maxA;
            float maxL;

            const double kGravityThreshold = 0.05;
            const float kThrottle = 0.75f;
        }

        //V0.1
        internal sealed class PeripheryControl
        {
            public PeripheryControl(IMyGridTerminalSystem grid_sys, IMyTerminalBlock source, IMyShipController ctrl)
            {
                connectors = new List<IMyShipConnector>();
                grid_sys.GetBlocksOfType<IMyShipConnector>(connectors, b => b.IsSameConstructAs(source));

                gears = new List<IMyLandingGear>();
                grid_sys.GetBlocksOfType<IMyLandingGear>(gears, b => b.IsSameConstructAs(source));

                vents = new List<IMyAirVent>();
                grid_sys.GetBlocksOfType<IMyAirVent>(vents, b => b.IsSameConstructAs(source));

                doors = new List<IMyDoor>();
                grid_sys.GetBlocksOfType<IMyDoor>(doors, b => b.IsSameConstructAs(source));

                controller = ctrl;
            }

            public bool IsLanded()
            {
                bool rv = false;
                foreach (IMyLandingGear gear in gears)
                    rv |= gear.IsLocked;
                return rv;
            }

            public bool IsConnected()
            {
                bool rv = false;
                foreach (IMyShipConnector connector in connectors)
                    rv |= (connector.Status == MyShipConnectorStatus.Connected);
                return rv;
            }

            public void Undock()
            {
                foreach (IMyDoor door in doors)
                    door.CloseDoor();
                foreach (IMyShipConnector connector in connectors)
                    connector.Disconnect();
                foreach (IMyLandingGear gear in gears)
                    gear.Unlock();
                if (controller == null)
                    return;
                if (controller.IsUnderControl)
                    foreach (IMyAirVent vent in vents)
                    {
                        vent.Enabled = true;
                        if (vent.CanPressurize)
                            vent.Depressurize = false;
                    }
            }

            IMyShipController controller;
            List<IMyShipConnector> connectors;
            List<IMyLandingGear> gears;
            List<IMyAirVent> vents;
            List<IMyDoor> doors;
        };

        //V0.1
        internal sealed class FuelControl
        {
            public FuelControl(IMyGridTerminalSystem grid_sys, IMyTerminalBlock source)
            {
                generators = new List<IMyGasGenerator>();
                grid_sys.GetBlocksOfType<IMyGasGenerator>(generators, b => b.IsSameConstructAs(source));

                tanks = new List<IMyGasTank>();
                grid_sys.GetBlocksOfType<IMyGasTank>(tanks, b => b.IsSameConstructAs(source));
            }

            public void Start()
            {
                foreach (IMyGasGenerator generator in generators)
                    generator.Enabled = true;
                foreach (IMyGasTank tank in tanks)
                {
                    tank.Enabled = true;
                    tank.Stockpile = false;
                }
            }

            public void Stop()
            {
                foreach (IMyGasGenerator generator in generators)
                    generator.Enabled = false;
            }

            List<IMyGasGenerator> generators;
            List<IMyGasTank> tanks;
        };

        #region PreludeFooter
    }
}
#endregion