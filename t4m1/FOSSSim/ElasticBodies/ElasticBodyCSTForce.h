#ifndef __ELASTIC_BODY_CST_FORCE_H__
#define __ELASTIC_BODY_CST_FORCE_H__

#include <Eigen/Core>
#include "../Force.h"
#include <iostream>

class ElasticBodyCSTForce : public Force
{
public:
    
    ElasticBodyCSTForce( int idx1, int idx2, int idx3, const scalar& youngsmodulus, const scalar& poissonratio, const Vector2s& xb1, const Vector2s& xb2, const Vector2s& xb3 );
    
    virtual ~ElasticBodyCSTForce();
    
    virtual void addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E );
    
    virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE );
    
    virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE );
    
    virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, MatrixXs& hessE );
    
    virtual Force* createNewCopy();
    
private:
    int m_idx1;
    int m_idx2;
    int m_idx3;
    
    scalar m_youngs_modulus;
    scalar m_poisson_ratio;
    
    Vector2s m_xb1;
    Vector2s m_xb2;
    Vector2s m_xb3;
};

#endif
