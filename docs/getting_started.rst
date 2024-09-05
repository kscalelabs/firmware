Getting Started
===============

Instructions to quickly set up and run the firmware.

Installation
------------

Standard Installation
^^^^^^^^^^^^^^^^^^^^^

To install the package, run:

.. code-block:: bash

   pip install -e .

Installation for Jetson
^^^^^^^^^^^^^^^^^^^^^^^

1. Install Conda:

   .. code-block:: bash

      wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
      chmod +x Miniforge3-Linux-aarch64.sh
      ./Miniforge3-Linux-aarch64.sh
      source ~/.bashrc

2. Create Conda environment and install package:

   .. code-block:: bash

      conda create --name firmware python=3.11
      conda activate firmware
      make install-dev

Working with CAN
--------------------

1. Set up the CAN bus (this might already be happening in a `systemctl` service):

   .. code-block:: bash

      sudo ip link set can0 up type can bitrate 1000000
      sudo ip link set can1 up type can bitrate 1000000
      sudo ip link set can... up type can bitrate 1000000
      sudo ifconfig can0 txqueuelen 65536
      sudo ifconfig can1 txqueuelen 65536
      sudo ifconfig can... txqueuelen 65536

Development
-----------

To test C++ code and bindings quickly, run:

.. code-block:: bash

   make build-ext
