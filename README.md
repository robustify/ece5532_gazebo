# ece5532_gazebo
ROS 2 packages to support ECE 5532 examples and homework assignments

## Blender Notes

For each model in the `models` directory, there is a Blender file that is used to export the `.obj` and `.mtl` files that are loaded into Gazebo.
The Z offset of the road segments is designed to extrude out from the Gazebo ground plane by 5mm so there isn't a hard edge on the roads in the simulator. Make sure the Z offset of the geometry in Blender is -0.045m relative to its origin.

When exporting from Blender, be sure to:

- Export --> Wavefront (.obj)
    - Set the forward axis to `+X` and the up axis to `+Z`
    - Make sure the `Materials` checkbox is selected
    - Make sure the path mode is set to `Relative`