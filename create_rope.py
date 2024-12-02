import pybullet as p
import pybullet_data


def create_rope(num_segments, start_pos, rope_params:dict):

    rope_length = rope_params["length"]
    num_segments = rope_params["num_segments"]
    segment_length = rope_length / num_segments
    segment_radius = rope_params["segment_radius"]
    segment_mass = rope_params["segment_mass"]
    rope_segments = []

    # Create rope segments
    for i in range(num_segments):
        # pos = [start_pos[0] + i * segment_length, start_pos[1], start_pos[2] ]
        pos = [start_pos[0] , start_pos[1], start_pos[2] + i * segment_length]


        # Create capsule segment
        segment_col_shape = p.createCollisionShape(
            shapeType=p.GEOM_CAPSULE, radius=segment_radius, height=segment_length
        )
        segment_vis_shape = p.createVisualShape(
            shapeType=p.GEOM_CAPSULE,
            radius=segment_radius,
            length=segment_length,
            rgbaColor=[1, 0, 0, 1],
        )
        segment_id = p.createMultiBody(
            baseMass=segment_mass,
            baseCollisionShapeIndex=segment_col_shape,
            baseVisualShapeIndex=segment_vis_shape,
            basePosition=pos,
        )

        # Disable collisions between adjacent segments for stability
        if i > 0:
            p.setCollisionFilterPair(
                segment_id, rope_segments[i - 1], -1, -1, enableCollision=0
            )
        rope_segments.append(segment_id)

    # Add constraints between rope segments
    for i in range(num_segments - 1):
        p.createConstraint(
            parentBodyUniqueId=rope_segments[i],
            parentLinkIndex=-1,
            childBodyUniqueId=rope_segments[i + 1],
            childLinkIndex=-1,
            jointType=p.JOINT_POINT2POINT,
            jointAxis=[0, 0, 0],
            # parentFramePosition=[segment_length / 2, 0, 0],
            # childFramePosition=[-segment_length / 2, 0, 0],
            parentFramePosition=[0, 0, segment_length / 2],
            childFramePosition=[0, 0, -segment_length / 2],
        )

    return rope_segments