# Wrench generation package for active constraints 

This package contains a ros node that generates wrenches (forces + torques) 
based on the current position of a tool and its desired one. Torques are 
generated as a simple visco-elastic function of the orientation. To generate 
forces 4 methods are available: a visco-elastic method and three 
_non-energy-storing_ force generation methods. For a description of these 
methods and discussions on their behavior please refer to:

_"N. Enayati, E. C. Alves Costa, G. Ferrigno, and E. De Momi, “A Dynamic 
Non-Energy-Storing Guidance Constraint with Motion Redirection for 
Robot-Assisted Surgery,” in IEEE/RSJ International Conference on Intelligent 
Robots and Systems, IROS, 2016"_  

If this package turns out to be useful to you, please cite the above paper in
your works.

ALthough the package was used with a da Vinci research kit, the implementation
of the wrench generation methods is accessible as a separate C++ library 
(ActiveConstraintEnforcementMethods.cpp/.hpp) that can be used with any c++ 
application as long as KDL types are defined. 


## License

This software is released under a BSD license:

    Software License Agreement (BSD License)
    Copyright (c) 2016, Nima Enayati


    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of nearlab nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   -
