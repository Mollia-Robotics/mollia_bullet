mollia_bullet
=============

.. rubric:: Version - |version| - |commit|

World
-----

.. autofunction:: mollia_bullet.world

Simulation
^^^^^^^^^^

.. automethod:: mollia_bullet.World.simulate

Contacts
^^^^^^^^

.. automethod:: mollia_bullet.World.contacts_of
.. automethod:: mollia_bullet.World.contacts_between
.. automethod:: mollia_bullet.World.contacts_between2

Helpers
^^^^^^^

.. automethod:: mollia_bullet.World.helper
.. automethod:: mollia_bullet.World.contact_helper

Updaters
^^^^^^^^

Updaters are simple callable objects with no parameter.
Updaters are called after every simulation step.

.. automethod:: mollia_bullet.World.add_updater
.. automethod:: mollia_bullet.World.remove_updater

Objects
^^^^^^^

.. automethod:: mollia_bullet.World.box
.. automethod:: mollia_bullet.World.sphere
.. automethod:: mollia_bullet.World.hinge
.. automethod:: mollia_bullet.World.fixed
.. automethod:: mollia_bullet.World.motor_control
.. automethod:: mollia_bullet.World.group

MotorControl
------------

Reset
^^^^^

.. automethod:: mollia_bullet.MotorControl.reset

reset must be called after transforming the objects.
velocity will be zero on the first frame after reset.

Data
^^^^

.. automethod:: mollia_bullet.MotorControl.position
.. automethod:: mollia_bullet.MotorControl.velocity

Group
-----

Transform
^^^^^^^^^

.. automethod:: mollia_bullet.Group.apply_transform
.. automethod:: mollia_bullet.Group.apply_force
.. automethod:: mollia_bullet.Group.apply_torque

Save Load
^^^^^^^^^

.. automethod:: mollia_bullet.Group.save_state
.. automethod:: mollia_bullet.Group.load_state

.. note:: Do not forget to reset the MotorControl.

Data
^^^^

.. automethod:: mollia_bullet.Group.aabb
.. automethod:: mollia_bullet.Group.center_of_mass

Visualization
^^^^^^^^^^^^^

.. automethod:: mollia_bullet.Group.color_mesh
.. automethod:: mollia_bullet.Group.transforms

Box
---

.. autoattribute:: mollia_bullet.Box.mass
.. autoattribute:: mollia_bullet.Box.world
.. autoattribute:: mollia_bullet.Box.constraints
.. autoattribute:: mollia_bullet.Box.groups
.. autoattribute:: mollia_bullet.Box.color
.. autoattribute:: mollia_bullet.Box.visible
.. autoattribute:: mollia_bullet.Box.group
.. autoattribute:: mollia_bullet.Box.mask
.. autoattribute:: mollia_bullet.Box.size

.. autoattribute:: mollia_bullet.Box.origin
.. autoattribute:: mollia_bullet.Box.basis
.. autoattribute:: mollia_bullet.Box.stiffness
.. autoattribute:: mollia_bullet.Box.contact_stiffness
.. autoattribute:: mollia_bullet.Box.contact_damping
.. autoattribute:: mollia_bullet.Box.contact_stiffness_flag
.. autoattribute:: mollia_bullet.Box.linear_friction
.. autoattribute:: mollia_bullet.Box.rolling_friction
.. autoattribute:: mollia_bullet.Box.spinning_friction

.. automethod:: mollia_bullet.Box.apply_force
.. automethod:: mollia_bullet.Box.apply_torque
.. automethod:: mollia_bullet.Box.contacts
.. automethod:: mollia_bullet.Box.penetration
.. automethod:: mollia_bullet.Box.config
.. automethod:: mollia_bullet.Box.remove

Sphere
------

.. autoattribute:: mollia_bullet.Sphere.mass
.. autoattribute:: mollia_bullet.Sphere.world
.. autoattribute:: mollia_bullet.Sphere.constraints
.. autoattribute:: mollia_bullet.Sphere.groups
.. autoattribute:: mollia_bullet.Sphere.color
.. autoattribute:: mollia_bullet.Sphere.visible
.. autoattribute:: mollia_bullet.Sphere.group
.. autoattribute:: mollia_bullet.Sphere.mask
.. autoattribute:: mollia_bullet.Sphere.radius

.. autoattribute:: mollia_bullet.Sphere.origin
.. autoattribute:: mollia_bullet.Sphere.basis
.. autoattribute:: mollia_bullet.Sphere.stiffness
.. autoattribute:: mollia_bullet.Sphere.contact_stiffness
.. autoattribute:: mollia_bullet.Sphere.contact_damping
.. autoattribute:: mollia_bullet.Sphere.contact_stiffness_flag
.. autoattribute:: mollia_bullet.Sphere.linear_friction
.. autoattribute:: mollia_bullet.Sphere.rolling_friction
.. autoattribute:: mollia_bullet.Sphere.spinning_friction

.. automethod:: mollia_bullet.Sphere.apply_force
.. automethod:: mollia_bullet.Sphere.apply_torque
.. automethod:: mollia_bullet.Sphere.contacts
.. automethod:: mollia_bullet.Sphere.penetration
.. automethod:: mollia_bullet.Sphere.config
.. automethod:: mollia_bullet.Sphere.remove

Hinge
-----

.. autoattribute:: mollia_bullet.Hinge.world
.. autoattribute:: mollia_bullet.Hinge.body_a
.. autoattribute:: mollia_bullet.Hinge.body_b

.. automethod:: mollia_bullet.Hinge.remove

Fixed
-----

.. autoattribute:: mollia_bullet.Fixed.world
.. autoattribute:: mollia_bullet.Fixed.body_a
.. autoattribute:: mollia_bullet.Fixed.body_b

.. automethod:: mollia_bullet.Fixed.remove

Types
-----

.. autoclass:: mollia_bullet.World
.. autoclass:: mollia_bullet.Box
.. autoclass:: mollia_bullet.Sphere
.. autoclass:: mollia_bullet.Hinge
.. autoclass:: mollia_bullet.Fixed
.. autoclass:: mollia_bullet.MotorControl
.. autoclass:: mollia_bullet.Group

.. toctree::
   :maxdepth: 2
   :caption: Contents:
