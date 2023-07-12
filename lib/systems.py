from audioop import cross
from email.quoprimime import body_check
from pydrake.all import (LeafSystem_, AbstractValue,
                        ExternallyAppliedSpatialForce_,
                        TemplateSystem,
                        RigidTransform, RotationMatrix,
                        Cylinder, Sphere, DiagramBuilder,
                        LogVectorOutput,
                        Parser, JointIndex, RevoluteJoint,
                        AddMultibodyPlantSceneGraph,
                        Wing, SlenderBody, ConstantVectorSource,
                        PlanarJoint, FixedOffsetFrame,
                        Propeller, SpatialInertia, UnitInertia, Demultiplexer, Multiplexer, PassThrough)
import numpy as np

from .helpers import MakeNamedView

@TemplateSystem.define("SpatialForceConcatinator_")
def SpatialForceConcatinator_(T):
    class Impl(LeafSystem_[T]):
        def _construct(self, N_inputs, converter = None):
            LeafSystem_[T].__init__(self, converter)
            self.N_inputs = N_inputs
            self.Input_ports = [self.DeclareAbstractInputPort(f"Spatial_Force_{i}",
                                AbstractValue.Make([ExternallyAppliedSpatialForce_[T]()])) for i in range(N_inputs)]
        
            self.DeclareAbstractOutputPort("Spatial_Forces",
                                           lambda: AbstractValue.Make(
                                               [ExternallyAppliedSpatialForce_[T]()for i in range(N_inputs)]),
                                           self.Concatenate)

        def Concatenate(self, context, output):
            out = []
            for port in self.Input_ports:
                out += port.Eval(context)
            output.set_value(out)
        
        def _construct_copy(self, other, converter=None,):
            Impl._construct(self, other.N_inputs, converter=converter)
    
    return Impl


@TemplateSystem.define("Barometer_")
def Barometer_(T):
    class Impl(LeafSystem_[T]):
        def _construct(self, body_index, P0 = 101325.0, rho0 = 1.2250, T0 = 288.15,
                       L0 = -0.0065, h0 = 0.0, g = 9.80665, M = 0.0289644, converter = None):
            LeafSystem_[T].__init__(self, converter)
        
            self.body_index = body_index
            self.P0 = P0 #reference pressure (Pa)
            self.rho0 = rho0 #reference density (kg/m^3)
            self.T0 = T0 #reference temperature (K)
            self.L0 = L0 #temperature lapse rate (k/m)
            self.h0 = h0 #height of reference level (m)
            self.g = g #gravitational acceleration (m/s^2)
            self.M = M #molar mass of air (kg/mol)
        
            self.R = 9.31444598 # universal gas constant (J/(mol*K ))
        
        
            self.Pose_port = self.DeclareAbstractInputPort("body_poses_input_port",AbstractValue.Make([RigidTransform()]))
        
            self.DeclareVectorOutputPort("pressure", 1, self.CalcPressure)
            self.DeclareVectorOutputPort("density", 1, self.CalcDensity)
        
        def CalcPressure(self, context, output):
            h = self.Pose_port.Eval(context)[self.body_index].translation()[2]
        
            if self.L0 == 0:
                P = self.P0 * np.exp(-self.g * self.M * (h - self.h0) / (self.R * self.T0))
            else:
                P = self.P0 * (1 + (h - self.h0) * self.L0 / self.T0) ** (-self.g * self.M / (self.R * self.L0))
            output.SetFromVector(np.array([P]))
        
        def CalcDensity(self, context, output):
            h = self.Pose_port.Eval(context)[self.body_index].translation()[2]
        
            if self.L0 == 0:
                rho = self.rho0 * np.exp(-self.g * self.M * (h - self.h0) / (self.R * self.T0))
            else:
                rho = self.rho0 * (self.T0 / (self.T0 + (h - self.h0) * self.L0)) ** (1 + self.g * self.M / (self.R * self.L0))
            output.SetFromVector(np.array([rho]))
        
        def _construct_copy(self, other, converter=None,):
            Impl._construct(self, other.body_index, other.P0, other.rho0, other.T0, other.L0,
                            other.h0, other.g, other.M, converter=converter)
    
    return Impl

# Default instantations
#SpatialForceConcatinator = SpatialForceConcatinator_[None] 
#Barometer = Barometer_[None]

class Starship:
    def __init__(self, time_step = 0.001, planar = False, V_WWind = [0,0,0], COM_rocket = np.zeros(3), has_engine = False):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    
        starship_index = Parser(plant).AddModelFromFile("starship/urdf/starship.urdf", "starship")

        # Add joint actuators and populate limits
        self.PositionLowerLimit = []
        self.PositionUpperLimit = []
        #TODO velocity limits
        for ind in range(plant.num_joints()):
            #TODO Add acutator inertias
            joint = plant.get_joint(JointIndex(ind))
            if type(joint) == RevoluteJoint:
                plant.AddJointActuator(joint.name() + "_motor", joint) #infinite effort limit
                self.PositionLowerLimit.append(joint.position_lower_limit())
                self.PositionUpperLimit.append(joint.position_upper_limit())
        
        if planar:
            planar_joint_frame = plant.AddFrame(FixedOffsetFrame("planar_joint_frame", plant.world_frame(), RigidTransform(RotationMatrix.MakeXRotation(np.pi/2))))                                                  
            planar_body_frame = plant.AddFrame(FixedOffsetFrame("planar_body_frame", plant.GetFrameByName("body"), RigidTransform(RotationMatrix.MakeXRotation(np.pi/2))))                                                  
            joint = PlanarJoint("Joint_body", planar_joint_frame,  planar_body_frame)
            plant.AddJoint(joint)
        
        if has_engine:
            inertia = UnitInertia.SolidSphere(1.0)
            Link_Gimbal_x = plant.AddRigidBody("Link_Gimbal_x", starship_index, SpatialInertia(mass=0, p_PScm_E=[-21.65, 0., 0.], G_SP_E=inertia))
            Link_Gimbal_y = plant.AddRigidBody("Link_Gimbal_y", starship_index, SpatialInertia(mass=0, p_PScm_E=[0, 0., 0.], G_SP_E=inertia))
            gimbal_joint_frame = plant.AddFrame(FixedOffsetFrame("Link1_Gimbal_frame", plant.GetFrameByName("body"),
                                                RigidTransform(RotationMatrix.MakeYRotation(-np.pi/2), p = [-21.65, 0, 0])))                                                  

            Joint_Gimbal_x = plant.AddJoint(RevoluteJoint("Joint_Gimbal_x", gimbal_joint_frame,
                                            Link_Gimbal_x.body_frame(), [1,0,0]))
            Joint_Gimbal_y = plant.AddJoint(RevoluteJoint("Joint_Gimbal_y", Link_Gimbal_x.body_frame(),
                                            Link_Gimbal_y.body_frame(), [0,1,0]))
            plant.AddJointActuator(Joint_Gimbal_x.name() + "_motor", Joint_Gimbal_x)
            plant.AddJointActuator(Joint_Gimbal_y.name() + "_motor", Joint_Gimbal_y)

            #add visual gemoetires 
            plant.RegisterVisualGeometry(Link_Gimbal_y, RigidTransform(),
                                            Cylinder(radius = 2, length = 4), "engine_vis",
                                            np.array([0.5, 0.5, 0.5, 1.]))

            plant.RegisterVisualGeometry(Link_Gimbal_y, RigidTransform(p = [0.0, 0.0, 2.0]),
                                            Sphere(radius = 1.5), "fire_vis",
                                            np.array([1.0, 0.0, 0.0, 1.]))
        
        plant.AddFrame(FixedOffsetFrame("com_frame", plant.GetFrameByName("body"), RigidTransform(p = COM_rocket)))                                                  

        plant.Finalize()

        #### Add Aerodynamic models ####
        wind = builder.AddSystem(ConstantVectorSource(V_WWind))
        barometer = builder.AddSystem(Barometer_[None](body_index = plant.GetFrameByName("body").body().index()))

        self.Leg_Area = 47.859
        self.Arm_Area = 16.295

        #Assume that the main bodies center of pressure coincides with the center of mass of the rocket 
        self.COM_rocket = plant.CalcCenterOfMassPositionInWorld(plant.CreateDefaultContext())

        rocketbody = builder.AddSystem(SlenderBody(body_index = plant.GetFrameByName("body").body().index(), X_BodyCP = RigidTransform(p = self.COM_rocket), 
                                      transverse_surface_area =  61.164, longitudinal_surface_area = 395.82,
                                      transverse_CD = 0.42, longitudinal_CD = 1.2, fluid_density = 1.204)) 
 
        Leg_R = builder.AddSystem(Wing(body_index = plant.GetFrameByName("Link_Leg_R").body().index(),
                                        X_BodyWing = RigidTransform(p = [0,-1.88,0]),
                                          surface_area = self.Leg_Area, fluid_density = 1.204))

        Leg_L = builder.AddSystem(Wing(body_index = plant.GetFrameByName("Link_Leg_L").body().index(),
                                      X_BodyWing = RigidTransform(p = [0,1.88,0]),
                                      surface_area = self.Leg_Area, fluid_density = 1.204))

        Arm_R = builder.AddSystem(Wing(body_index = plant.GetFrameByName("Link_Arm_R").body().index(),
                                      X_BodyWing = RigidTransform(p = [0,-1.26,0]),
                                      surface_area = self.Arm_Area, fluid_density = 1.204))

        Arm_L = builder.AddSystem(Wing(body_index = plant.GetFrameByName("Link_Arm_L").body().index(),
                                      X_BodyWing = RigidTransform(p = [0,1.26,0]),
                                       surface_area = self.Arm_Area, fluid_density = 1.204))

        flaps = [Leg_R, Leg_L, Arm_R, Arm_L]
        
        if has_engine:
            #add propellor as engine in Mega newtons
            engine = builder.AddSystem(Propeller(plant.GetBodyByName("Link_Gimbal_x").index(),
                                        RigidTransform(RotationMatrix.MakeYRotation(-np.pi/2)), 10e6))
            demux = builder.AddSystem(Demultiplexer(7))
            mux = builder.AddSystem(Multiplexer(6))
            extra_ports = 2

        else:
            extra_ports = 1
        concat = builder.AddSystem(SpatialForceConcatinator_[None](len(flaps) + extra_ports))

        #connect Flaps
        for i, control_surface in enumerate(flaps):
            #inputs 
            builder.Connect(plant.get_body_poses_output_port(), control_surface.get_body_poses_input_port())
            builder.Connect(plant.get_body_spatial_velocities_output_port(), control_surface.get_body_spatial_velocities_input_port())

            builder.Connect(wind.get_output_port(0), control_surface.get_wind_velocity_input_port())
            #builder.Connect(barometer.GetOutputPort("density"), control_surface.get_fluid_density_input_port())
    
            #output
            builder.Connect(control_surface.get_spatial_force_output_port(), concat.get_input_port(i))
        
        #Connect rocket body
        builder.Connect(plant.get_body_poses_output_port(), rocketbody.get_body_poses_input_port())
        builder.Connect(plant.get_body_spatial_velocities_output_port(), rocketbody.get_body_spatial_velocities_input_port())
        builder.Connect(wind.get_output_port(0), rocketbody.get_wind_velocity_input_port())
        #builder.Connect(barometer.GetOutputPort("density"), rocketbody.get_fluid_density_input_port())

        builder.Connect(rocketbody.get_spatial_force_output_port(), concat.get_input_port(len(flaps)))
    
        #output
        builder.Connect(concat.get_output_port(0), plant.get_applied_spatial_force_input_port())


        #Export inputs and state
        builder.ExportOutput(plant.get_state_output_port(), "state")
        #builder.ExportOutput(plant.get_continuous_state_port(), "state")
        builder.ExportOutput(scene_graph.get_query_output_port(), "geometry_query")
 
        if has_engine:
            input = builder.AddSystem(PassThrough(7))
            
            builder.Connect(plant.get_body_poses_output_port(), engine.get_body_poses_input_port())
            builder.Connect(engine.get_spatial_forces_output_port(), concat.get_input_port(len(flaps) + 1))

            for i in range(6):
                builder.Connect(demux.get_output_port(i), mux.get_input_port(i))

            builder.Connect(demux.get_output_port(6), engine.get_command_input_port())
            builder.Connect(mux.get_output_port(), plant.get_actuation_input_port())

            builder.Connect(input.get_output_port(), demux.get_input_port())

            builder.ExportInput(input.get_input_port(), "input")
        else:
            input = builder.AddSystem(PassThrough(4))
            builder.Connect(input.get_output_port(), plant.get_actuation_input_port())
            builder.ExportInput(input.get_input_port(), "input")


        self.X_logger = LogVectorOutput(plant.get_state_output_port(), builder)
        self.U_logger = LogVectorOutput(input.get_output_port(), builder)

        diagram = builder.Build()
        
        self.diagram_ = diagram
        self.plant_ = plant

        self.named_view_ = MakeNamedView(self.plant_)

        self.planar = planar

    def GetDiagram(self):
        return self.diagram_
    
    def GetPlant(self):
        return self.plant_

    def GetLogs(self, context):
        return self.StateView(self.X_logger.FindLog(context).data()), self.U_logger.FindLog(context).data()

    def CreateDefaultContext(self):
        return self.diagram_.CreateDefaultContext()
    
    def StateView(self, q):
        return self.named_view_(q)
    
    def init(self, context, angles = [np.pi/2, np.pi/2, np.pi/2, np.pi/2], leg_angle = 1.57, **kwargs):
        """
            angles are ordered by Joint_Leg_L, Joint_Leg_R, Joint_Arm_L, Joint_Arm_R
        """
        if self.planar:
            joint_pos = kwargs["joint_pos"]
            joint_vel = kwargs["joint_vel"]

            self.plant_.GetJointByName("Joint_body").set_translation(context, joint_pos[:2])
            self.plant_.GetJointByName("Joint_body").set_translational_velocity(context, joint_vel[:2])

            self.plant_.GetJointByName("Joint_body").set_rotation(context, joint_pos[2])
            self.plant_.GetJointByName("Joint_body").set_angular_velocity(context, joint_vel[2])
        else:
            X_WB = kwargs["X_WB"]
            V_WB = kwargs["V_WB"]
            self.plant_.SetFreeBodyPose(context,
                               self.plant_.GetBodyByName("body"), X_WB)
            self.plant_.SetFreeBodySpatialVelocity(context=context,
                                          body=self.plant_.GetBodyByName("body"),
                                          V_WB=V_WB)


        self.plant_.GetJointByName("Joint_Arm_L").set_angle(context, angles[0])
        self.plant_.GetJointByName("Joint_Arm_R").set_angle(context, angles[1])
        self.plant_.GetJointByName("Joint_Leg_L").set_angle(context, angles[2])
        self.plant_.GetJointByName("Joint_Leg_R").set_angle(context, angles[3])