//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2016, Nearlab


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
*/
//==============================================================================

//------------------------------------------------------------------------------

#include "ActiveConstraintEnforcementMethods.hpp"

using namespace toolbox;


//-----------------------------------------------------------------------
// SIMULATED PLASTICY INTRODUCED BY KIKUUWE ET AL. 2008
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
acPlast::acPlast(const double F_MAX,
                 const double ELASTIC_LENGTH,
                 const double BOUNDARY_THRESHOLD ){
    F_MAX_ 				= F_MAX;
    ELASTIC_LENGTH_ 		= ELASTIC_LENGTH;
    BOUNDARY_THRESHOLD_ 	= BOUNDARY_THRESHOLD;
    p_tool_last_ 		= KDL::Vector(0.0,0.0,0.0);
    q_					= KDL::Vector(0.0,0.0,0.0);

}

//-----------------------------------------------------------------------
// FORCE GENERATION FOLLOWING THE EQUATIONS DESCRIBED IN THE
// CORRESPONDING PAPER
//-----------------------------------------------------------------------

void acPlast::getForce(KDL::Vector &f_out,
                       const KDL::Vector p_tool,
                       const KDL::Vector p_desired,
                       const KDL::Vector v_msrd){

    double F = 0.5;
    double R = F_MAX_;
    // TODO: Kk was int, not sure why. changed it to double and has to be tested
    double Kk = R/ELASTIC_LENGTH_;
    double Bk = 1.5;
    double T = 0.001;
    KDL::Vector cp = p_desired;

    KDL::Vector q_last = q_;
    double cte = Kk + (Bk/T);
    KDL::Vector ps = p_tool + (Bk * (q_ - p_tool_last_) / (Kk * T + Bk)); // p_tool-d = p_last
    KDL::Vector ki = ps - cp;
    KDL::Vector n = ki;
    normalizeSafe(n, KDL::Vector(0.0,0.0,0.0));

    double a1 = saturate(0.0, dot(n,ki), (R/cte));
    double a2 = saturate(0.0, -dot(n,ki), (R/cte));

    KDL::Vector e = q_last - ps;
    double a3 = saturate(-a1, dot(n,e), a2);

    //I-n*n'
    KDL::Vector In1 = KDL::Vector(1 - n[0]*n[0], 0 - n[0]*n[1], 0 - n[0]*n[2]);
    KDL::Vector In2 = KDL::Vector(0 - n[1]*n[0], 1 - n[1]*n[1], 0 - n[1]*n[2]);
    KDL::Vector In3 = KDL::Vector(0 - n[2]*n[0], 0 - n[2]*n[1], 1 - n[2]*n[2]);
    //(I-n*n')*e
    KDL::Vector mat = KDL::Vector(dot(In1,e), dot(In2,e), dot(In3,e));
    double m = std::max(1.0,( ( cte/F ) * sqrt( ( e.Norm()*e.Norm() ) - ( dot(n,e)*dot(n,e) ) ) ) );

    q_ = ps + n * a3 + mat/m;

    KDL::Vector a4 = saturate_vec(e, (F/cte));
    KDL::Vector qf = ps + a4;
    double qfql = (qf - q_last).Norm();
    double qql = (q_ - q_last).Norm();

    if ( qfql <= qql ) q_ = qf;

    f_out = (Kk * (q_ - p_tool) + Bk * ( ( q_ - q_last ) - ( p_tool - p_tool_last_ ) ) / T);

    p_tool_last_ = p_tool;
}


//-----------------------------------------------------------------------
// PLASTIC WITH MOTION REDIRECTION - INTRODUCED BY BOWYER ET AL. 2013
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
acPlastRedirect::acPlastRedirect(const double F_MAX,
                                 const double ELASTIC_LENGTH,
                                 const double BOUNDARY_THRESHOLD)
{
    F_MAX_ = F_MAX;
    ELASTIC_LENGTH_ =ELASTIC_LENGTH;
    BOUNDARY_THRESHOLD_=BOUNDARY_THRESHOLD;
}

//-----------------------------------------------------------------------
// FORCE GENERATION FOLLOWING THE EQUATIONS DESCRIBED IN THE
// CORRESPONDING PAPER
//-----------------------------------------------------------------------
void acPlastRedirect::getForce(KDL::Vector &f_out,
                               const KDL::Vector p_tool,
                               const KDL::Vector p_desired,
                               const KDL::Vector v_msrd){

    double sig2 = 2.5;
    double fc = F_MAX_;
    double theta = 0.4;
    double sig0 = fc/ELASTIC_LENGTH_;
    double zcss = fc/sig0;
    KDL::Vector penet = p_tool - p_desired ;

    // boundary condition
    double ptrans = BOUNDARY_THRESHOLD_;
    if ( penet.Norm() < ptrans ) theta = theta * (penet.Norm()/ptrans);

    z_ = z_ + p_tool- p_tool_last_;

    KDL::Vector ptn = penet;
    normalizeSafe(ptn, KDL::Vector(0.0,0.0,0.0));
    KDL::Vector zn = z_;
    normalizeSafe(zn, KDL::Vector(0.0,0.0,0.0));
    double a = atan2((ptn*zn).Norm(), dot(zn, ptn));

    KDL::Vector n = penet*z_;
    KDL::Vector nn = n;
    normalizeSafe(nn, KDL::Vector(0.0,0.0,0.0));

    KDL::Vector y = cos(theta)*penet + sin(theta)*(nn*penet) + (1-cos(theta))*dot(nn,penet)*nn;
    KDL::Vector yn = y;
    normalizeSafe(yn, KDL::Vector(0.0,0.0,0.0));

    if ( a <= theta && z_.Norm() <= zcss )
        z_ = z_ * 1;
    else if ( a <= theta && z_.Norm() > zcss )
        z_ = zcss*zn;
    else
    {
        if ( dot(z_,yn) <= 0.0 ) z_ = KDL::Vector(0.0,0.0,0.0);
        else if ( (0.0 < dot(z_,yn)) && (dot(z_,yn) < zcss) ) z_ = dot(z_,yn)*yn;
        else z_ = zcss * yn;
    }

    f_out =  -(sig0*z_ + sig2*v_msrd);
    p_tool_last_ = p_tool;
}



//-----------------------------------------------------------------------
// VISCOUSE WITH REDIRECTION - INTRODUCED BY ENAYATI ET AL. 2016
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
acViscousRedirect::acViscousRedirect(double F_MAX_in,
                                     double B_MAX_in,
                                     double BOUNDARY_THRESHOLD_in){

    F_MAX_ = F_MAX_in;
    B_MAX_ = B_MAX_in;
    BOUNDARY_THRESHOLD_ = BOUNDARY_THRESHOLD_in;


    v_dir_last_ = KDL::Vector(1.0,0.0,0.0);
    f_dir_last_ = KDL::Vector(1.0,0.0,0.0);
    n_2_last_   = KDL::Vector(1.0,0.0,0.0);

}

//-----------------------------------------------------------------------
// FORCE GENERATION FOLLOWING THE EQUATIONS DESCRIBED IN THE
// CORRESPONDING PAPER
//-----------------------------------------------------------------------

void acViscousRedirect::getForce(KDL::Vector &f_out,
                                 const KDL::Vector p_tool,
                                 const KDL::Vector p_desired,
                                 const KDL::Vector v_msrd){

    // UPPERCASE NAMES = SCALARS
    // LOWERCASE NAMES = VECTORS

    KDL::Vector penet = p_desired - p_tool;
    double F_VC, F_VC_SAT;
    double B_M			= B_MAX_;

    KDL::Vector  f_dir, penet_dir, v_tool_dir;

    penet_dir 		= penet;
    v_tool_dir 		= v_msrd;

    normalizeSafe(v_tool_dir, v_dir_last_);
    normalizeSafe(penet_dir, KDL::Vector(1.0,0.0,0.0));

    double V_PENET_DOTP = dot(v_tool_dir, penet_dir);
    double PENET =penet.Norm();

    // limit the viscous coefficient around the boundary to minimize the oscillation
    if (PENET< BOUNDARY_THRESHOLD_){
        B_M =  B_MAX_ * PENET/ BOUNDARY_THRESHOLD_;
    }
    else
        B_M = B_MAX_;

    // calculate the magnitude of the ac force
//	F_VC = B_M * sqrt( ( 1 - V_PENET_DOTP ) / 2 ) * v_msrd.Norm();
    F_VC = B_M * sqrt( ( 1 - V_PENET_DOTP ) / 2 ) * v_msrd.Norm();
    // saturate the magnitude of the ac force
    F_VC_SAT = saturate(0.0, F_VC , F_MAX_ );

    //-----------------------------------------------------------------------
    // calculate the direction of the AC force
    KDL::Vector n = v_tool_dir * penet_dir;
    KDL::Vector nn = n;
    normalizeSafe(nn, penet_dir);

    double THETA = (M_PI/2) * (1 + V_PENET_DOTP);

    if ( V_PENET_DOTP < 0.0 )
        f_dir = penet_dir;
    else
    if(rotateVector(v_tool_dir, f_dir, nn, THETA) < 0)
        std::cout << "Null in rotateVector." <<"  norm(v_tool_dir) = "<< v_tool_dir.Norm()<<
                  "  , norm(nn) = "<< nn.Norm() << std::endl;

    //-----------------------------------------------------------------------
    // Make the force vector from the calculated magnitude and direction
    f_out = F_VC_SAT * f_dir;

    v_dir_last_ = v_tool_dir;

}




//-----------------------------------------------------------------------
// SIMPLE ELASTIC WITH DAMPING
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
acElastic::acElastic(const double F_MAX,
                     const double TAW_MAX,
                     const double k,
                     const double b,
                     const double kappa,
                     const double c)
{
    F_MAX_ = F_MAX;
    TAW_MAX_ = TAW_MAX;
    k_ = k;
    b_ = b;
    kappa_ = kappa;
    c_ = c;
//    penet_vel_ = 0;

};


//-----------------------------------------------------------------------
// FORCE GENERATION
//-----------------------------------------------------------------------
void acElastic::getForce(KDL::Vector &f_out,
                         const KDL::Vector p_tool,
                         const KDL::Vector p_desired,
                         const KDL::Vector twist_msrd){

    // find the penetration vector
    KDL::Vector penet = p_desired - p_tool;

    // make the force vector
    KDL::Vector f_all = k_ * penet - b_ * twist_msrd;
    double f_all_magnitude = f_all.Norm();

    // limit the force to F_MAX_
    if (f_all_magnitude > F_MAX_)
        f_out = F_MAX_/f_all_magnitude * f_all;
    else
        f_out = f_all;

//    // save last penet
//    penet_last = penet;

}


//-----------------------------------------------------------------------
// FORCE TORQUE
//-----------------------------------------------------------------------
void acElastic::getTorque(KDL::Vector &taw_out,
                          const KDL::Rotation rot_current,
                          const KDL::Rotation rot_desired,
                          const KDL::Vector rot_vel){

    // find the delta rotation
    KDL::Rotation d_rot = rot_desired * rot_current.Inverse();

    // convert to RPY
    KDL::Vector delta_rpy;
    d_rot.GetRPY(delta_rpy[0], delta_rpy[1], delta_rpy[2]);

    // find the elastic torque
    KDL::Vector taw_all = kappa_ * delta_rpy - c_ * rot_vel;
    double taw_norm = taw_all.Norm();

    // limit the force to F_MAX_
    if (taw_norm > TAW_MAX_)
        taw_out = TAW_MAX_/taw_norm * taw_all;
    else
        taw_out = taw_all;

}




//##############################################################################
// #############################################################################
// #################       COMMON FUNCTIONS         ############################
// #############################################################################
//##############################################################################

int toolbox::rotateVector(const KDL::Vector vector_in,
                          KDL::Vector &vector_out,
                          const KDL::Vector n,
                          const double THETA){

    if (vector_in.Norm() ==0.0 || n.Norm() ==0.0){
//		ROS_ERROR("Null vector given as the input of rotation function.");
        return -1;
    }
    else{
        vector_out = vector_in + sin(THETA)*(n*vector_in)+(1.0-cos(THETA))*(n*(n*vector_in));
        return 0;
    }
}
int toolbox::normalizeSafe(KDL::Vector & vec_in,
                           const KDL::Vector vec_safe) {

    // Checks the norm of the vec_in. If it is non zero the function normalizes it.
    // If the norm is zero the function uses vec_safe as the output

    double l = vec_in.Norm();
    if( fabs(l) >= std::numeric_limits<double>::epsilon() ) {
        vec_in[0] /= l;
        vec_in[1] /= l;
        vec_in[2] /= l;
        return 0;
    } else {

        vec_in = vec_safe;
        return -1;
    }
}

double toolbox::saturate (double a,
                          double x,
                          double b){

    if (x < a){
        return a;
    }
    else if (x >b){
        return b;
    }
    else
        return x;

}

KDL::Vector toolbox::saturate_vec(KDL::Vector x,
                                  double a){

    double l = x.Norm();
    KDL::Vector xn = x;
    normalizeSafe(xn, KDL::Vector(0.0,0.0,0.0));

    if (l <= a) return x;
    else return a * xn;

}




