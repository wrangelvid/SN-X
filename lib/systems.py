from pydrake.all import (LeafSystem_, AbstractValue,
                        ExternallyAppliedSpatialForce, TemplateSystem,
                        RigidTransform)
import numpy as np

@TemplateSystem.define("SpatialForceConcatinator_")
def SpatialForceConcatinator_(T):
    class Impl(LeafSystem_[T]):
        def _construct(self, N_inputs, converter = None):
            LeafSystem_[T].__init__(self, converter)
            self.N_inputs = N_inputs
            self.Input_ports = [self.DeclareAbstractInputPort(f"Spatial_Force_{i}",
                                AbstractValue.Make([ExternallyAppliedSpatialForce()])) for i in range(N_inputs)]
        
            self.DeclareAbstractOutputPort("Spatial_Forces",
                                           lambda: AbstractValue.Make(
                                               [ExternallyAppliedSpatialForce()for i in range(N_inputs)]),
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
        
            self.DeclareVectorOutputPort("Pressure", 1, self.CalcPressure)
            self.DeclareVectorOutputPort("Density", 1, self.CalcDensity)
        
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

SpatialForceConcatinator = SpatialForceConcatinator_[None] 
Barometer = Barometer_[None]