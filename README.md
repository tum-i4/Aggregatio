# Knowledge Aggregation in Multi-Agent Self-Adaptive Cyber-Physical Systems

This repository contains the source code and resources to perform Knowledge Aggregation with various probabilistic logics, such as Subjective Logic, in a ROS-based Simulated Self-Adaptive Multi-Robot System.

## Workspace Structure

The [ros](../ros) folder contains the ROS implementation of a Multi-Robot system. This ROS-based testbed consists of a set of Python and C++ ROS nodes that allow for an easy experimentation with the knowledge aggregation using the different available fusion operators in subjective logic.

The [subjective_logic](../subjective_logic) directory contains the source code that implements the formalisms of subjective logic; in particular, it provides an API to perform the aggregation of the knowledge of multiple agents using different fusion operators. 

The folders themselves contain a more detailed explanation about their content. In particular, the [ros](../ros) folder incorporates a detailed step by step guide on how to install and run the system, also including architectural details of the system.
