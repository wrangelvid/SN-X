from pydrake.all import (PlanarJoint, JointIndex, namedview)

def MakeNamedView(plant):
       names_pos = [None]*plant.num_positions()
       for ind in range(plant.num_joints()):
           joint = plant.get_joint(JointIndex(ind))
           if type(joint) == PlanarJoint:
               names_pos[joint.position_start()]     = joint.name() + "_x"
               names_pos[joint.position_start() + 1] = joint.name() + "_y"
               names_pos[joint.position_start() + 2] = joint.name() + "_theta"
           else:
               names_pos[joint.position_start()] = joint.name() + "_p"
       for ind in plant.GetFloatingBaseBodies():
           body = plant.get_body(ind)
           start = body.floating_positions_start()
           body_name = body.name()
           names_pos[start] = body_name+'_qw'
           names_pos[start+1] = body_name+'_qx'
           names_pos[start+2] = body_name+'_qy'
           names_pos[start+3] = body_name+'_qz'
           names_pos[start+4] = body_name+'_x'
           names_pos[start+5] = body_name+'_y'
           names_pos[start+6] = body_name+'_z'
       
       names_vel = [None]*plant.num_velocities()
       for ind in range(plant.num_joints()):
           joint = plant.get_joint(JointIndex(ind))
           if type(joint) == PlanarJoint:
               names_vel[joint.velocity_start()]     = joint.name() + "_vx"
               names_vel[joint.velocity_start() + 1] = joint.name() + "_vy"
               names_vel[joint.velocity_start() + 2] = joint.name() + "_vtheta"
           else:
               names_vel[joint.velocity_start()] = joint.name() + "_v"
       for ind in plant.GetFloatingBaseBodies():
           body = plant.get_body(ind)
           start = body.floating_velocities_start() - plant.num_positions()
           body_name = body.name()
           names_vel[start] = body_name+'_wx'
           names_vel[start+1] = body_name+'_wy'
           names_vel[start+2] = body_name+'_wz'
           names_vel[start+3] = body_name+'_vx'
           names_vel[start+4] = body_name+'_vy'
           names_vel[start+5] = body_name+'_vz'
       
       return namedview("state", names_pos + names_vel)
   