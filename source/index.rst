.. RT2-first documentation master file, created by
   sphinx-quickstart on Tue Apr  1 18:06:21 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to RT2-first's documentation!
=====================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Research_Track_assignment's documentation!
==========================================

This is the documentation of the Research_Track1_assignment!

---

node_a
------

Node A orchestrates target setting and motion control. It integrates an action client for setting navigation targets and publishes real-time kinematic data, including position and velocity, derived from the robot's odometry.

.. automodule:: scripts.node_a
   :members:
   :undoc-members:
   :show-inheritance:
   :imported-members:

---

node_b
------

Node B serves as a memory bank for the system, offering a service to recall the last target position set by the user. It efficiently retrieves and delivers this historical data upon request, ensuring seamless continuity in navigation tasks.

.. automodule:: scripts.node_b
   :members:
   :undoc-members:
   :show-inheritance:
   :imported-members:

---

node_c
------

Node C serves as an analytical hub, dynamically computing the average velocity and tracking the distance to the last target position. By processing real-time velocity data and leveraging historical target positions, it provides valuable insights for fine-tuning robot navigation strategies.

.. automodule:: scripts.node_c
   :members:
   :undoc-members:
   :show-inheritance:
   :imported-members:



