Simulating Moose
=================

Whether you actually have a Moose robot or not, the Moose simulator is a great way to get started with ROS
robot development. In this tutorial, we will go through the basics of starting Gazebo and Rviz and how to drive
your Moose around.


Installation
------------

To get started with the Moose simulation, make sure you have a :roswiki:`working ROS installation <ROS/Installation>`
set up on your Ubuntu desktop, and install the Moose-specific metapackages for desktop and simulation:

.. code:: bash

    sudo apt-get install ros-kinetic-moose-simulator ros-kinetic-moose-desktop


Launch Gazebo
-------------

Gazebo is the most common simulation tool used in ROS. Moose's model in Gazebo include reasonable
approximations of its dynamics, including wheel slippage, skidding, and inertia. To launch simulated
Moose in a simple example world, run the following command:

.. code-block:: bash

    roslaunch moose_gazebo moose_world.launch

You should see the following window appear, or something like it. You can adjust the camera angle by
clicking and dragging while holding CTRL, ALT, or the shift key:

.. image:: graphics/moose_simulator_gazebo.png
    :alt: Simulated Moose in the Race World.

The window which you are looking at is the Gazebo Client. This window shows you the "true" state of the
simulated world which the robot exists in. It communicates on the backend with the Gazebo Server, which
is doing the heavy lifting of actually maintaining the simulated world. At the moment, you're running
both the client and server locally on your own machine, but some advanced users may choose to run heavy
duty simulations on separate hardware and connect to them over the network.


Launch rviz
-----------

The next tool we will encounter is :roswiki:`rviz`. Although superficially similar in appearance to Gazebo,
rviz has a very different purpose— unlike Gazebo, which shows the reality of the simulated world, rviz shows
the robot's *perception* of its world, whether real or simulated. So while Gazebo won't be used with your
real Moose, rviz is used with both.

You can using the following launch invocation to start rviz with a pre-cooked configuration suitable for
visualizing any standard Moose config:

.. code-block:: bash

    roslaunch moose_viz view_robot.launch

You should see rviz appear:

.. image:: graphics/moose_viz.png
    :alt: Moose with laser scanner in rviz.

The rviz display only shows what the robot knows about its world, which presently, is nothing. Because the
robot doesn't yet know about the barriers which exist in its Gazebo world, they're not shown here.


Driving Moose
--------------

What is shown, however, is Moose's interactive markers. These are the simplest way to command your robot
to move around. If you don't see them in your rviz display, select the Interact tool from the top toolbar.
You should see red arrows and a blue circle appear around the Moose model.

Drag the red arrows in Rviz to move in the linear x and the blue circle to move in the angular z. Rviz shows you
Moose moving relative to its odometric frame, but it is also moving relative to the simulated world supplied by
Gazebo. If you click over to the Gazebo window, you will see Moose moving within its simulated world. Or, if you
drive real Moose using this method, it will have moved in the real world.

Once you start your own development, have your nodes send ``geometry_msgs/Twist`` commands to the ``cmd_vel``
topic to drive Moose, either real or simulated. This is the standard ROS interface to differential-drive and
holonomic ground vehicles.


Visualizing Sensors
-------------------

The rviz tool is capable of visualizing many common robotic sensors, as well as other data feeds which can give
us clues as to what the robot is doing and why. A great place to start with this is adding the
:roswiki:`LaserScan <rviz/DisplayTypes/LaserScan>` plugin to visualize the laser scans being produced by the
simulated LMS111. In the left panel, click the "Add" button, then select the "Topics" tab, and then select the
``front/scan`` topic:

.. image:: rviz-visualize-laser.png
    :alt: Adding a laser scan visualization to Moose.

Click OK, and you should see laser scan points now visible in the rviz window, relative to the robot:

.. image:: rviz-laser-scan.png
    :alt: Visualizing Moose with simulated laser scans.

If you use the interactive markers to drive around, you'll notice that the laser scan points move a little bit
but generally stay where they are. This is the first step toward map making using :roswiki:`gmapping`, which
is covered in the next tutorial, :doc:`navigation`.
