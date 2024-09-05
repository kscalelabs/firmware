Testing and Debugging
=====================

This section will guide you through testing and debugging the firmware.

Testing CAN Bus
---------------

After setting up the CAN bus, you can use the `candump` utility to monitor CAN messages:

.. code-block:: bash

   candump can0

To check if your CAN bus is working, you can send messages between two CAN buses in a pseudo-loopback setup:

To do this, physically connect two CAN buses together. Then, send a message from one bus to the other:

.. code-block:: bash

   candump can0

.. code-block:: bash

   cansend can1 123#FEEDBACC

You should see the message echoed back on the other bus:

Testing Robstride Motors
------------------------

For a quick and dirty test of the Robstride motors, you can use the `robstride` CLI:

.. code-block:: bash

   pip install robstride

The Robstride CLI can be used to read, write, and update the CAN ID of the Robstride Motors.

Example usage:

.. code-block:: bash

   robstride --interface socketcan --channel can0 enable [can_id]
   robstride --interface socketcan --channel can0 read [can_id] loc_ref
   robstride --interface socketcan --channel can0 write [can_id] loc_ref [value]
   robstride --interface socketcan --channel can0 update_id [can_id] [new_id]
   robstride --interface socketcan --channel can0 disable [can_id]
