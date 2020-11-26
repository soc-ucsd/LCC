# LCC
Leading Cruise Control is a general control framework for connected and autonomous vehicles (CAVs) in mixed traffic flow, where human-driven vehicles (HDVs) also exist.
<img src="docs/img/LCC.png" align="center" width="100%"/>
The blue arrows represent the communication topology of the CAV, while the purple arrows illustrate the interaction direction in HDVs' dynamics. The blue vehicles, gray vehicles and yellow vehicles represent CAVs, HDVs and the head vehicle, respectively.

Two special cases of LCC are **Car-Driving LCC (CF-LCC)** and **Free-Driving LCC (FD-LCC)**.
<img src="docs/img/FD-CF.png" align="center" width="100%"/>
<img src="docs/img/CFLCC.gif" align="center" width="49.5%"/>
<img src="docs/img/FDLCC.gif" align="center" width="49.5%"/>

## Features
The CAV maintains car-following operations, adapting to the states of its preceding vehicles, and also aims to **lead the motion of its following vehicles**. Specifically, by controlling of the CAV, LCC aims to attenuate downstream traffic perturbations and **smooth upstream traffic flow actively**.

## Publications
Wang, J., Zheng, Y., Chen, C., Xu, Q., & Li, K. (2020). Leading cruise control in mixed traffic flow. 59th IEEE Conference on Decision and Control, 2020. Available: https://arxiv.org/abs/2007.11753

The presentation slides can be downloaded [here](https://wangjw18.github.io/files/2020-CDC-slides.pdf).

## Contacts
Relavent project: [MCMT](https://github.com/wangjw18/MCMT) (modeling and control of mixed traffic flow).
