#ifndef __ELASTIC_BODY_SPRING_FORCE_H__
#define __ELASTIC_BODY_SPRING_FORCE_H__

#include <Eigen/Core>
#include "../Force.h"
#include <iostream>

class ElasticBodySpringForce : public Force
{
public:
    
    ElasticBodySpringForce( int idx1, int idx2, const scalar& alpha, const scalar& l0);
    
    virtual ~ElasticBodySpringForce();
    
    virtual void addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E );
    
    virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE );
    
    virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE );
    
    virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE );
    
    virtual Force* createNewCopy();
    
private:
    int m_idx1;
    int m_idx2;
    
    scalar m_alpha; // stiffness coefficient
    scalar m_l0;    // rest length
};

#endif
