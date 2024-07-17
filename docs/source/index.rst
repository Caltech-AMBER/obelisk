.. obelisk documentation master file, created by
   sphinx-quickstart on Fri Jun 14 15:35:25 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Documentation for ``obelisk``
=============================
.. TODO: Fix the TOC so all the pages have the same one!

.. toctree::
   :maxdepth: 2
   :caption: Table of Contents:
   :glob:

   getting_started.md
   development.md
   obelisk_api.md
   using_obelisk.rst
   visualization.md
   logging.md
   obelisk_terminal_aliases.md
   faq.md

Overview
--------

``obelisk`` is a shared robot control interface for the robots in the `AMBER Lab at Caltech <http://www.bipedalrobotics.com/>`_. Welcome!

The goal of ``obelisk`` is to provide a lightweight, convenient API built on ROS2 that forms a barebones stack for robotic systems of interest. Some elements of the project that we emphasize:

* Hardware deployment should be identical or as close as possible to simulated deployment. That is, by simply switching a flag, we should be able to visualize and test code in simluation and then immediately run the same code on hardware seamlessly.
* The block diagram for every system follows a standard: a *model* (either the hardware or sim) exists which we want to control, which sends information back to a *state estimator*, which sends an observation to the *controller*, which executes a control action. The focus of the end-user should be on designing elements of the controller or state estimator - once the system model is specified, it should abstractly be able to interface with any compatible controller or estimator. There are also *sensor* blocks which represent offboard sensing elements (such as external cameras), which provide additional signal to the state estimator.
* In particular, end-user development should be *agnostic* to our ROS2 code, which only thinly wraps control code and allows different elements of the stack to communicate with each other.
* Unified logging and visualization tools are important for understanding system behavior and debugging. These utilities are provided in ``obelisk``.
