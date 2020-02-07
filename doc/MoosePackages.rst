Configuration & Environment Variables
=========================================

The moose_description package is the URDF robot description for Moose UGV.

.. _Source: https://github.com/moose-cpr/moose


Overview
---------

This package provides a `URDF <http://wiki.ros.org/urdf>`_ model of Moose.  For an example launchfile to use in visualizing this model, see `moose_viz <http://wiki.ros.org/moose_viz>`_.

.. image:: graphics/moose_urdf_banner.png


Accessories
------------

Moose has a suite of optional payloads called accessories. These payloads can be enabled and placed on the robot using environment variables specified at the time the `xacro <http://wiki.ros.org/xacro>`_ is rendered to URDF. Available accessory vars are:

.. raw:: html

    <table><tbody><tr> <td><p><strong>Variable</strong> </p></td>
      <td><p><strong>Default</strong> </p></td>
      <td><p><strong>Description</strong> </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_CONTROL_EXTRAS</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_GENERATOR</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_IMU_XYZ</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_JOY_TELEOP</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_JOY_TELEOP_CONTROL</tt> </p></td>
      <td><p><tt>false</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_NAVSAT_SMART6</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_NAVSAT_SMART6_BAUD</tt> </p></td>
      <td><p><tt>57600</tt> </p></td>
      <td><p>Sets the baud rate for serial communication with the GPS module</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_NAVSAT_SMART6_MOUNT</tt> </p></td>
      <td><p><tt>navsat</tt> </p></td>
      <td><p>The mount on the robot model that the GPS antenna is mounted to.  See the Moose URDF and <tt>MOOSE_URDF_EXTRAS</tt> for more details on mount points.</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_NAVSAT_SMART6_PORT</tt> </p></td>
      <td><p><tt>/dev/ttyS1</tt> </p></td>
      <td><p>The serial port that the GPS module communicates over</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_OFFBOARD_STOP</tt> </p></td>
      <td><p><tt>false</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_TRACKS</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_TWIST_MUX_EXTRAS</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>MOOSE_URDF_EXTRAS</tt> </p></td>
      <td><p><tt>empty.urdf</tt> </p></td>
      <td><p>Path to a URDF file with additional modules connected to the robot</p></td>
    </tr>
    </tbody></table>

Configurations
----------------

As an alternative to individually specifying each accessory, some fixed configurations are provided in the package. These can be specified using the ``config arg to description.launch``, and are intended especially as a convenience for simulation launch.

====================================  ====================================================
Config:                               Description:
====================================  ====================================================
base                                  Base Moose, includes IMU and GPS
arm_mount                             Includes mounting points for am arm payload
bulkhead                              ??? TODO
====================================  ====================================================
