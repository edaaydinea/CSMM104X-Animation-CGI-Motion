#ifndef __ELASTIC_BODY_BENDING_FORCE_H__
#define __ELASTIC_BODY_BENDING_FORCE_H__

#include <Eigen/Core>
#include "../Force.h"
#include <iostream>

class ElasticBodyBendingForce : public Force
{
public:
    
    ElasticBodyBendingForce( int idx1, int idx2, int idx3, const scalar& alpha, const scalar& theta0, const scalar& eb1n, const scalar& eb2n );
    
    virtual ~ElasticBodyBendingForce();
    
    virtual void addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E );
    
    virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE );
    
    virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE );
    
    virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE );
    
    virtual Force* createNewCopy();
    
private:
    int m_idx1;
    int m_idx2;
    int m_idx3;
    
    scalar m_alpha;     // stiffness coefficient
    scalar m_theta0;    // rest angle
    scalar m_eb1n;      // norm of e1 bar
    scalar m_eb2n;      // norm of e2 bar
};

#endif
