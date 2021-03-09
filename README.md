# Framework for Knowledge Aggregation with Subjective Logic in Multi-Agent Self-Adaptive Cyber-Physical Systems

This repository contains the source code and resources to perform Knowledge Aggregation with Subjective Logic in a ROS-based Simulated Self-Adaptive Multi-Robot System.

## Academic Citations

Please cite the following paper when using this tool.

Petrovska, Ana, et al. "Knowledge aggregation with subjective logic in multi-agent self-adaptive cyber-physical systems." Proceedings of the IEEE/ACM 15th International Symposium on Software Engineering for Adaptive and Self-Managing Systems. 2020.

@inproceedings{DBLP:conf/icse/PetrovskaQGP20,
  author    = {Ana Petrovska and
               Sergio Quijano and
               Ilias Gerostathopoulos and
               Alexander Pretschner},
  editor    = {Shinichi Honiden and
               Elisabetta Di Nitto and
               Radu Calinescu},
  title     = {Knowledge aggregation with subjective logic in multi-agent self-adaptive
               cyber-physical systems},
  booktitle = {{SEAMS} '20: {IEEE/ACM} 15th International Symposium on Software Engineering
               for Adaptive and Self-Managing Systems, Seoul, Republic of Korea,
               29 June - 3 July, 2020},
  pages     = {149--155},
  publisher = {{ACM}},
  year      = {2020},
  url       = {https://doi.org/10.1145/3387939.3391600},
  doi       = {10.1145/3387939.3391600},
  timestamp = {Tue, 29 Dec 2020 18:35:13 +0100},
  biburl    = {https://dblp.org/rec/conf/icse/PetrovskaQGP20.bib},
  bibsource = {dblp computer science bibliography, https://dblp.org}
}

## Workspace Structure

The [ros](../ros) folder contains the ROS implementation of a Multi-Robot system. This ROS-based testbed consists of a set of Python and C++ ROS nodes that allow for an easy experimentation with the knowledge aggregation using the different available fusion operators in subjective logic.

The [subjective_logic](../subjective_logic) directory contains the source code that implements the formalisms of subjective logic; in particular, it provides an API to perform the aggregation of the knowledge of multiple agents using different fusion operators. 

The folders themselves contain a more detailed explanation about their content. In particular, the [ros](../ros) folder incorporates a detailed step by step guide on how to install and run the system, also including architectural details of the system.
