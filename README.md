# Affordance Primitives

This is the high-level repository for Affordance Primitives (APs). For further details see:
  1. [affordance_primitives](affordance_primitives/) for the core code and documentation
  2. [affordance_primitive_msgs](affordance_primitive_msgs/) for AP message definitions
  3. [ap_examples](ap_examples/) for examples using APs in your own work

# Installation
Note that the `main` branch is on ROS2 Rolling. For Noetic, use the `noetic` branch.

To install and build `Affordance Primitives`:
```sh
git clone https://github.com/UTNuclearRobotics/affordance_primitives.git
rosdep install --from-paths affordance_primitives/ --ignore-src -y
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Citation
This work was developped by the Nuclear and Applied Robotics Group at The University of Texas at Austin.

The core of this work was published in
```
@article{pettinger2022versatile,
  title={A Versatile Affordance Modeling Framework Using Screw Primitives to Increase Autonomy During Manipulation Contact Tasks},
  author={Pettinger, Adam and Alambeigi, Farshid and Pryor, Mitch},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={3},
  pages={7224--7231},
  year={2022},
  publisher={IEEE}
}
```
