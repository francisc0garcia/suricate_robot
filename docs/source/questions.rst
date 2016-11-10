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

under development...

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


Other questions
^^^^^^^^^^^^^^^

* How can I monitor the performance of my Raspberry PI 2 or ODROID:

Answer: you can monitor remotely in real-time the RAM and CPU consumption using SSH, open two different
terminal windows and run:

.. code-block:: none

    watch -n 5 free -m
    htop





