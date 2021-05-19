Welcome to the documentation of ros2_socket_bridge!
===================================================

This is a package developed for ROS2 'Foxy' which allows the utilization of Python sockets in ROS2, with the main goal being transmission of topics between domains. It can be used for multiple purposes, and could add an additional communication layer on top of ROS-implemented DDS. 

An example use case being a control domain and multiple robot domains. The domains important to monitor the robots are transmitted from the robot domain to the control domain, and tasks are sent to each robot individually. This would allow for easier control of robot fleets with less namespace clutter and setup.

Contents:
^^^^^^^^^

.. toctree::
   :maxdepth: 2
   
   qs_guide
   internal_workings
   testing
   license
