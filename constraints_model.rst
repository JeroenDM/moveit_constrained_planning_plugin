=====================
constraints\_model.md
=====================

.. role:: math(raw)
   :format: html latex
..

Path constraints
================

This document explains how I plan to model end-effector constraints to
integrate OMPL's `constrained planning
library <http://ompl.kavrakilab.org/constrainedPlanning.html>`__ into
MoveIt. Many of it is based on the current `implementation of
constraints in
MoveIt <https://github.com/ros-planning/moveit/blob/d03bb752e55e3e399541cb773cb389e6da5e27f8/moveit_core/kinematic_constraints/src/kinematic_constraint.cpp>`__,
and on the `paper by Berenson et.
al. <https://journals.sagepub.com/doi/abs/10.1177/0278364910396389>`__.

Everything is an equality constraint
------------------------------------

OMPL's constrained planning interface requires us to write the
constraints in a generic form :math:`D(q) = 0`. The vector
:math:`q \in \mathbb{R}^n ` holds the :math:`n` joint positions. With
:math:`m` constraints, :math:`D` is vector valued function
:math:`[ d_0(q)\; d_1(q)\; \dots \; d_m(q)]^T` that maps
:math:`\mathbb{R}^n \rightarrow \mathbb{R}^m`.

Position constraints
~~~~~~~~~~~~~~~~~~~~

Consider we want to keep the end-effector's :math:`x`-position in
between :math:`0.2` and :math:`0.3`. A straight forward way to define
the corresponding function :math:`d_i(q)` is as follows:

.. math::

   \begin{align}
   d_i(x) = 
   \begin{cases}
   0 &\text{if} &0.2 \le x \le 0.3 \\
   0.2 - x &\text{if} &x \lt 0.2 \\
   x - 0.3 &\text{if} &x \gt 0.3
   \end{cases}
   \end{align}

or expressed graphically as:

where :math:`d_i` is the :math:`i^{th}` constraint in :math:`D`. To get
:math:`d_i(q)` in function of the joint angles, we also need to forward
kinematics for the robot, in this case only for the :math:`x`-position,
:math:`f_x(q)`. The the equality constraint equation becomes:

.. math:: d_i(f_x(q)) = 0

with the function :math:`d_i(...)` shown in the figure above.

If we are given an actual equality constraint, we convert it to bounds
using a specified tolerance :math:`\delta`. For example on the
end-effector's :math:`y`-position, :math:`y = 0.6`, is interpreted as
:math:` (0.6 - \delta/2) \lt y \lt (0.6 + \delta/2)`. This keeps
everything nice and uniform. An alternative is to use it directly as
:math:`y - 0.6 = 0`. I'm not sure about the pros and cons of this
approach.

Position constraints in MoveIt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In MoveIt, we can use a
`SolidPrimitive.msg <http://docs.ros.org/melodic/api/shape_msgs/html/msg/SolidPrimitive.html>`__
to specify position constraints. The field ``dimensions`` can be used
for :math:`x`, :math:`y` and :math:`z` position tolerance. Although we
have to add a convention to specify no constraints on a specific value.
I use a value ``-1`` to indicate no constraints for now.

.. code:: python

    # Define box, sphere, cylinder, cone 
    # The type of the shape (use BOX=1 for position constraints here)
    uint8 type

    # The dimensions of the shape  (use -1 for an unconstrained dimension)
    float64[] dimensions

This is part of the
`BoundingVolume.msg <http://docs.ros.org/melodic/api/moveit_msgs/html/msg/BoundingVolume.html>`__
where we also specify the reference position in the field
``primitive_poses``. This bounding volume is then put into the field
``constraint_region`` of a
`PositionConstraint.msg <http://docs.ros.org/melodic/api/moveit_msgs/html/msg/PositionConstraint.html>`__.

Orientation constraints
~~~~~~~~~~~~~~~~~~~~~~~

Expressing the end-effector's orientation constraints requires a choice
of parameterization. In code, an orientation is mostly saved as a unit
quaternion or a rotation matrix :math:`R`. In MoveIt code that works
with constraints, the rotation matrix is converted to roll pitch yaw
angles, written here as :math:`\alpha`, :math:`\beta` and
:math:`\gamma`. An orientation is expressed as a sequence of three
elementary rotations.

.. math:: R = R_X(\alpha) R_Y(\beta) R_Z(\gamma)

These angles are calculated for the relative orientation between a
desired orientation and the current orientation of the end-effector.

.. math::

   \begin{align}
   R_{error} &= R_{desired}^T R_{ee} \\
   R_{error} & \rightarrow [\alpha_{error}, \beta_{error}, \gamma_{error}]
   \end{align}

Now we can apply bounds to these angles as we did for position
constraints, using a similar function :math:`d(...)`. But in this case,
the bounds are expressed symmetrically around zero error. For example,
given and absolute tolerance of :math:`0.4` radians on :math:`\alpha`,
we would add the constraint :math:`-0.2 \lt \alpha_{error} \lt 0.2`.
Another constraints could be to have zero error on :math:`\beta`, which
is expressed as :math:`-\delta \lt \beta_{error} \lt \delta`. The
tolerance value :math:`\delta` should probably be different for position
and orientation.

Orientation constraints in MoveIt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The tolerances on the roll pitch yaw angles are specified in an
`OrientationConstraint.msg <http://docs.ros.org/melodic/api/moveit_msgs/html/msg/OrientationConstraint.html>`__,
again using ``-1`` to indicate and unconstrained angle.

.. code:: python

    # The desired orientation of the robot link specified as a quaternion
    geometry_msgs/Quaternion orientation

    # optional axis-angle error tolerances specified
    float64 absolute_x_axis_tolerance
    float64 absolute_y_axis_tolerance
    float64 absolute_z_axis_tolerance

Jacobian :math:`\partial D(q) / \partial q`
-------------------------------------------

Ideally, we also supply the Jacobian of the constraints to OMPL. In
MoveIt we can calculate the geometric Jacobian :math:`J_e(q)` . For the
:math:`x`-position constrained the first row of the Jacobian,
:math:`\partial f_x(q) / \partial q` is used to express the Jacobian of
the constraints. Notice the minus sign on the second row, because of the
derivative :math:`d_i(x)` being :math:`-1` for :math:`x \lt 0.2`.

.. math::

   \begin{align}
   \frac{\partial d_i(q)}{\partial q} = 
   \begin{cases}
   0 &\text{if} &0.2 \le x \le 0.3 \\
   -\partial f_x(q) / \partial q &\text{if} &x \lt 0.2 \\
   \partial f_x(q) / \partial q &\text{if} &x \gt 0.3
   \end{cases}
   \end{align}

For the orientation constraint we need an extra transformation of the
last three rows of the geometric Jacobian, which expresses the
relationship between joint velocity :math:`\dot{q}` and angular velocity
:math:`\omega`. Constraints are expressed using roll pitch yaw angles
:math:`\alpha`, :math:`\beta` and :math:`\gamma`. Therefore we need the
following conversion matrix:

.. math::

   \begin{bmatrix} \dot{\alpha} \\ \dot{\beta} \\ \dot{\gamma} \end{bmatrix} =
   E^{-1}
   \begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}

The expression uses :math:`E^{-1}`, because :math:`E` always exists, but
could be singular. This occurs at the well known singularity of three
angle representation of orientation. In this case
:math:`\beta = \pm \pi / 2`, as can be seen in the expression for
:math:`B^{-1}` derived in `these lecture
notes <https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>`__.

.. math::

   E^{-1}(\alpha, \beta) = 
   \begin{bmatrix}
   1 & \frac{sin(\alpha) sin(\beta)}{cos(\beta)} & \frac{-cos(\alpha) sin(\beta)}{cos(\beta)} \\
   0 & cos(\alpha) & sin(\alpha) \\
   0 & \frac{-sin(\alpha)}{cos(\beta)} & \frac{cos(\alpha)}{cos(\beta)}
   \end{bmatrix}

Now we can calculate the relevant analytical Jacobian :math:`J_a` from
the geometric Jacobian :math:`J_e` calculated by MoveIt as:

.. math::

   J_a(q) =
   \begin{bmatrix}
   I_{3x3} & 0_{3x3} \\
   0_{3x3} & E^{-1}
   \end{bmatrix}
   J_e(q)

and use the rows of :math:`J_a` to calculate the Jacobian of the
constraints. Note that the first three rows remain unchanged. In the
actual implementation we can just multiply the last three rows of
:math:`J_e` by :math:`E^{-1}`.

Implementation
--------------

A logical approach would be to add the :math:`m` constraints and create
an
`ompl::base::Constraint <http://ompl.kavrakilab.org/classompl_1_1base_1_1Constraint.html>`__
class with dimensions :math:`(n, m)`.

Another approach could be to fix the dimension on :math:`(n, 6)` and add
infinite bounds on unconstrained parameters. This makes the
implementation slightly easier. However, this would mean there will be
superfluous calculations involved, making this approach less efficient.

An third alternative is to fix the dimension on :math:`(n, 1)`, defining
:math:`D(q)` as:

.. math:: D(q) = \sum_{i=1}^{m} d_i(q)

maybe this can make the implementation even simpler, I don't know this
yet.

Recap
-----

A constraint is specified by a symmetric interval :math:`\Delta x`
around a reference value :math:`x_{ref}`.

.. math::

   \begin{align}
   x_{ref} - \frac{\Delta x}{2} \le x \le x_{ref} + \frac{\Delta x}{2} \\
   \end{align}

Where the error :math:`e` is used to simplify the expression.

.. math::

   \begin{align}
   \frac{\Delta x}{2} \le x - x_{ref} \le \frac{\Delta x}{2}\\
   -\Delta e \le e \le \Delta e
   \end{align}

The inequalities are converted into a single equality constraint using a
penalty function :math:`d(e)`:

.. math::

   \begin{align}
   d(e) = 
   \begin{cases}
   0 &\text{if} &|e| \le \Delta \\
   |e -\Delta e| &\text{if} &|e| \gt \Delta
   \end{cases}
   \end{align}

The Jacobian of the constraints is straightforward to calculate based on
the geometric Jacobian of the robot, available in MoveIt. The parameter
:math:`x` can be a position or an angle, in which case it gets a little
bit more complicated to implement.
