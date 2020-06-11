# Constrained planning plugin for MoveIt, using OMPL

This repository is supposed to become an MVP to validate that we can use the constrained planning stuff in OMPL through a simple interface.
I'm not sure if it will work.

## Notes

I would like to separate the MoveIt and OMPL code as much as possible. So someone who understands MoveIt but not OMPL can understand the MoveIt part and visa versa.

## TODO

- Use `Eigen::Isometry::linear()` instead of `Eigen::Isometry3d::rotation()` whenever possible. This is more efficient (and can be used when the rotation matrix is known to be valid?).
