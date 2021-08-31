#include "StableFluidsSim.h"
#include <Eigen/LU>

scalar lerp(scalar a, scalar b, scalar x) {
    return (1 - x) * a + x * b;
}

void StableFluidsSim::diffuseD(int N, ArrayXs * x, ArrayXs * x0, scalar diff, scalar dt)
{
  assert((*x0 == *x0).all());
  
  scalar a = diff * dt * N * N;
  *x = *x0;
  
  scalar dD;
  for (int k = 0; k < 20; k++)
  {
    for (int i = 1; i <= N; i++)
    {
      for (int j = 1; j <= N; j++) // IMPORTANT: DO NOT MODIFY THE LOOP ORDER
      {
        // STUDENTS: You will certainly need code here, do diffuse for ([1, N], [1, N])
        dD = a * ((*x)(i - 1, j) + (*x)(i + 1, j) + (*x)(i, j - 1) + (*x)(i, j + 1));
        (*x)(i, j) = ((*x0)(i, j) + dD) / (1 + 4 * a); 
      }
    }
  }
}

void StableFluidsSim::diffuseU(int N, ArrayXs * x, ArrayXs * x0, scalar diff, scalar dt)
{
    assert((*x0 == *x0).all());

    scalar a = diff * dt * N * N;
    *x = *x0;
    scalar du;
    int nTerms;
    for (int k = 0; k < 20; k++)
    {
        for (int i = 1; i <= N; i++)
        {
            for (int j = 0; j <= N; j++) // IMPORTANT: DO NOT MODIFY THE LOOP ORDER
            {
                // STUDENTS: You will certainly need code here, do diffuse for ([1, N], [0, N]), note the case when (j == 0) or (j == N) need special treatment
                if (j == 0) {
                    du = a * ((*x)(i - 1, j) + (*x)(i + 1, j) + (*x)(i, j + 1));
                    nTerms = 3;
                }
                else if (j == N) {
                    du = a * ((*x)(i - 1, j) + (*x)(i + 1, j) + (*x)(i, j - 1));
                    nTerms = 3; 
                }
                else {
                    du = a * ((*x)(i - 1, j) + (*x)(i + 1, j) + (*x)(i, j - 1) + (*x)(i, j + 1));
                    nTerms = 4;
                }
                (*x)(i, j) = ((*x0)(i, j) + du) / (1 + nTerms * a); 
            }
        }
    }
}

void StableFluidsSim::diffuseV(int N, ArrayXs * x, ArrayXs * x0, scalar diff, scalar dt)
{
    assert((*x0 == *x0).all());

    scalar a = diff * dt * N * N;
    *x = *x0;
    scalar du;
    int nTerms;
    for (int k = 0; k < 20; k++)
    {
        for (int i = 0; i <= N; i++)
        {
            for (int j = 1; j <= N; j++) // IMPORTANT: DO NOT MODIFY THE LOOP ORDER
            {
                if (i == 0) {
                    du = a * ((*x)(i + 1, j) + (*x)(i, j - 1) + (*x)(i, j + 1));
                    nTerms = 3;
                }
                else if (i == N) {
                    du = a * ((*x)(i - 1, j) + (*x)(i, j - 1) + (*x)(i, j + 1));
                    (*x)(i, j)= ((*x0)(i, j) + du) / (1 + 3 * a); 
                    nTerms = 3;
                }
                else {
                    du = a * ((*x)(i - 1, j) + (*x)(i + 1, j) + (*x)(i, j - 1) + (*x)(i, j + 1));
                    nTerms = 4;
                }
                (*x)(i, j) = ((*x0)(i, j) + du) / (1 + nTerms * a); 
            }
        }
    }
}

void StableFluidsSim::advectD(int N, ArrayXs * x, ArrayXs * x0, ArrayXs * u, ArrayXs * v, scalar dt)
{
    assert((*x0 == *x0).all());
    assert((*u == *u).all());
    assert((*v == *v).all());
    
    
    // STUDENTS: You will certainly need code here, advect for ([1, N], [1, N])
    scalar dt0 = dt * N;
    
    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j <= N; j++)
        {
            scalar i0 = i - dt0 * interpolateV(v, i, j);
            scalar j0 = j - dt0 * interpolateU(u, i, j);
            (*x)(i, j) = interpolateD(x0, i0, j0);
            
            
        }
    }
}

scalar StableFluidsSim::interpolateD(ArrayXs * d, scalar i, scalar j)
{
    // STUDENTS: You will certainly need code here, note the indices should be CLAMP-ed to [0, m_N], since we have to use (i + 1) and (j + 1)
    int i1 = CLAMP((int) i, 0, m_N);
    int i2 = i1 + 1;
    int j1 = CLAMP((int) j, 0, m_N);
    int j2 = j1 + 1;
    scalar s = CLAMP(i - i1, 0, 1);
    scalar t = CLAMP(j - j1, 0, 1);
    scalar d1 = lerp((*d)(i1, j1), (*d)(i2, j1), s);
    scalar d2 = lerp((*d)(i1, j2), (*d)(i2, j2), s);
    return lerp(d1, d2, t);
}

scalar StableFluidsSim::interpolateU(ArrayXs * u, scalar i, scalar j)
{
    // STUDENTS: You will certainly need code here, note the i index should be CLAMP-ed to [0, m_N], while j index should be CLAMP-ed to [0, m_N-1], since we have to use (i + 1) and (j + 1)
    int i1 = CLAMP((int) i, 0, m_N);
    int i2 = i1 + 1;
    int j1 = CLAMP((int) (j - 0.5), 0, m_N - 1);
    int j2 = j1 + 1;
    scalar s = CLAMP(i - i1, 0, 1);
    scalar t = CLAMP(j - j1 - 0.5, 0, 1);
    scalar u1 = lerp((*u)(i1, j1), (*u)(i2, j1), s);
    scalar u2 = lerp((*u)(i1, j2), (*u)(i2, j2), s);
    return lerp(u1, u2, t);
}

scalar StableFluidsSim::interpolateV(ArrayXs * v, scalar i, scalar j)
{
    // STUDENTS: You will certainly need code here
    int i1 = CLAMP((int) (i - 0.5), 0, m_N - 1);
    int i2 = i1 + 1;
    int j1 = CLAMP((int) j, 0, m_N);
    int j2 = j1 + 1;
    scalar s = CLAMP(i - i1 - 0.5, 0, 1);
    scalar t = CLAMP(j - j1, 0, 1);
    scalar v1 = lerp((*v)(i1, j1), (*v)(i2, j1), s);
    scalar v2 = lerp((*v)(i1, j2), (*v)(i2, j2), s);
    return lerp(v1, v2, t);
}

void StableFluidsSim::advectU(int N, ArrayXs * x, ArrayXs * x0, ArrayXs * u, ArrayXs * v, scalar dt)
{
    assert((*x0 == *x0).all());
    assert((*u == *u).all());
    assert((*v == *v).all());

    scalar dt0 = dt * N;
    for (int i = 1; i <= N; i++)
    {
        for (int j = 0; j <= N; j++)
        {
            // STUDENTS: You will certainly need code here,
            // add the origin of U grid to the coordinate before sampling, for example, sample at (i + 0, j + 0.5) when you need backtracing the old velocity at (i, j)
            scalar i0 = i - dt0 * interpolateV(v, i, j + 0.5);
            scalar j0 = j + 0.5 - dt0 * interpolateU(u, i, j + 0.5);
            // now you have the backward-traced velocity, minus it from the current position (i + 0, j + 0.5), then sample the velocity again.
            (*x)(i, j) = interpolateU(x0, i0, j0);
        }
    }
}

void StableFluidsSim::advectV(int N, ArrayXs * x, ArrayXs * x0, ArrayXs * u, ArrayXs * v, scalar dt)
{
    assert((*x0 == *x0).all());
    assert((*u == *u).all());
    assert((*v == *v).all());

    scalar dt0 = dt * N;
    for (int i = 0; i <= N; i++)
    {
        for (int j = 1; j <= N; j++)
        {
            // STUDENTS: You will certainly need code here
            scalar i0 = i + 0.5 - dt0 * interpolateV(v, i + 0.5, j);
            scalar j0 = j - dt0 * interpolateU(u, i + 0.5, j);
            (*x)(i, j) = interpolateV(x0, i0, j0);
        }
    }
}

void StableFluidsSim::project(int N, ArrayXs * u, ArrayXs * v, ArrayXs * u0, ArrayXs * v0)
{
    if (VERBOSE) std::cout << "u0: " << std::endl << *u0 << std::endl << std::endl;
    if (VERBOSE) std::cout << "v0: " << std::endl << *v0 << std::endl << std::endl;

    ArrayXs div(N + 2, N + 2);
    ArrayXs p(N + 2, N + 2);
    div.setZero();
    p.setZero();
    scalar h = 1.0 / N;

    // STUDENTS: You will certainly need code here

    // set solid boundary conditions, 0 the most top and bottom row / left and right column of u0, v0
    for (int i = 0; i <= N; ++i) {
        // Top and bottom rows
        (*u0)(0, i) = 0;
        (*u0)(N + 1, i) = 0;
        (*v0)(0, i) = 0;
        (*v0)(N, i) = 0;
        
        // Left and right columns
        (*u0)(i, 0) = 0;
        (*u0)(i, N) = 0;
        (*v0)(i, 0) = 0;
        (*v0)(i, N + 1) = 0;
        
    }
    // Edge cases
    (*v0)(0, N + 1) = 0;
    (*v0)(N, N + 1) = 0;
    (*u0)(N + 1, 0) = 0;
    (*u0)(N + 1, N) = 0;
    
    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j <= N; j++)
        {
          // compute divergence of the velocity field, note the divergence field is available from ([1, N], [1, N])
          div(i, j) = -h * ((*v0)(i, j) - (*v0)(i - 1, j) + (*u0)(i, j) - (*u0)(i, j - 1));
        }
    }

    for (int k = 0; k < 20; k++)
    {
        for (int i = 1; i <= N; i++)
        {
              for (int j = 1; j <= N; j++) // IMPORTANT: DO NOT MODIFY THE LOOP ORDER
              {
                  // solve for pressure inside the region ([1, N], [1, N])
                  int nTerms = 4;
                  scalar terms = p(i - 1, j) + p(i + 1, j) + p(i, j - 1) + p(i, j + 1);
                  
                  if (i == 1) {
                      terms -= p(i - 1, j);
                      --nTerms;
                  }
                  else if (i == N) {
                      terms -= p(i + 1, j);
                      --nTerms;
                  }
                  
                  if (j == 1) {
                      terms -= p(i, j - 1);
                      --nTerms;
                  }
                  else if (j == N) {
                      terms -= p(i, j + 1);
                      --nTerms;
                  }
                  p(i, j) = (div(i, j) + terms) / nTerms;
              }
        }
    }

    (*u) = (*u0);
    (*v) = (*v0);

    for (int i = 1; i <= N; i++)
    {
        for (int j = 1; j < N; j++)
        {
            // apply pressure to correct velocities ([1, N], [1, N)) for u, ([1, N), [1, N]) for v
            (*u)(i, j) -= (p(i, j + 1) - p(i, j)) / h;
            (*v)(j, i) -= (p(j + 1, i) - p(j, i)) / h;   
        }
    }
}