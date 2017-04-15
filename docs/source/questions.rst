Frequent questions
==================

This section explain some common problems and how to solve it!

Installation and compilation problems
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* If you want to change default GCC compiler to version 5, open a terminal and run:

.. code-block:: none

    export CC=/usr/bin/gcc-5
    export CXX=/usr/bin/g++-5

then you can compile using GCC 5.

ROS related problems
^^^^^^^^^^^^^^^^^^^^

* When I relaunch Raspberry PI2 nodes, sad face is showed by LED matrix.

Answer: sometimes, when you relaunch nodes, an instance of PIGPIO remain active and generate conflicts, you can stop
nodes and wait for couple of minutes before everything is closed by ubuntu, and then start again ros nodes.

Interface problems
^^^^^^^^^^^^^^^^^^

* When I close a node that uses gazebo, it takes too long before it closes.

Answer: You can close manually the process, run in a command window:

.. code-block:: none

    sudo killall gzserver gzclient


* How can I visualize the GPIO state of my raspberry PI 2 remotely?

Answer: You can use pigpio scope [http://abyz.co.uk/rpi/pigpio/piscope.html], install piscope in your PC and then run the following command in a terminal.


.. code-block:: none

    export PIGPIO_ADDR=IP_OF_RASPBERRY
    piscope

* I get error: initInitialise: bind to port 8888 failed (Address already in use)

Answer: This is caused by PIGPIO's daemon, you can stop all ros process, and kill its process using

.. code-block:: none

    sudo killall -9 pigpiod
    sudo rm /var/run/pigpio.pid

then wait some seconds before it is closed by ubuntu. After thath, you should be able to execute normally.


Other questions
^^^^^^^^^^^^^^^

* How can I monitor the performance of my Raspberry PI 2 or ODROID:

Answer: you can monitor remotely in real-time the RAM and CPU consumption using SSH, open two different
terminal windows and run:

.. code-block:: none

    watch -n 5 free -m
    htop

* How can I change router configuration?

Answer: You can connect to router, and open a web browser. Type **http://tplinklogin.net**, credentials are:
user **admin** and password **admin**.

You can, for instance, change static IP address assigned to RPI2, ODROID or PC (*DHCP/Address Reservation*). As well as checking if all devices are properly connected (*DHCP/DHCP Client List*)




