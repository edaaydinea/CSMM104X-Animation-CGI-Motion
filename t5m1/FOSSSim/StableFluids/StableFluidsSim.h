#ifndef __STABLE_FLUIDS_SIM_H__
#define __STABLE_FLUIDS_SIM_H__

#include <iostream>

#include "FOSSSim/MathUtilities.h"

class StableFluidsSim
{
public:
  
  // Initialize the fluid simulation. Assumes rows == cols.
  //   rows: Number of rows in the Eulerian grid.
  //   cols: Number of cols in the Eulerian grid.
  //   diff: Diffusion coefficient for the passive markers.
  //   visc: Viscosity of the fluid.
  StableFluidsSim( const int& rows, const int& cols, const scalar& diff = 0.0001, const scalar& visc = 0.00000001, bool use_advect = true, bool use_project = true );
  
  // Deallocates memory used during the simulation.
  ~StableFluidsSim();
  
  // Integrates the system forward in time by dt.
  virtual void stepSystem( const scalar& dt );
  
  // Returns an array containing the marker densities. Note that the
  // boundary of this array is padded to handle boundary conditions.
  const ArrayXs& getMarkerDensities() const;
  ArrayXs& getMarkerDensities();
  
  // Returns an array containing the horizontal components of the fluid velocity.
  // Note that the boundary of this array is padded to handle boundary conditions.
  ArrayXs& getHorizontalVelocities();
  const ArrayXs& getHorizontalVelocities() const;
  
  // Returns an array containing the vertical components of the fluid velocity.
  // Note that the boundary of this array is padded to handle boundary conditions.
  ArrayXs& getVerticalVelocities();
  const ArrayXs& getVerticalVelocities() const;
  
  // Returns the number of non-boundary rows.
  int physicalRows() const;
  // Returns the number of non-boundary columns.
  int physicalCols() const;
  
  // Clears the density and velocity fields
  virtual void clear();
  
  // Prescribed velocity field for debugging
  virtual void setPrescribedVelocity(int p);
  
  // Diffusion coef
  void setDiffusion(scalar diff);
  scalar getDiffusion();
  
  // Viscosity coef
  void setViscosity(scalar visc);
  scalar getViscosity();
  
  // use advection
  void setUseAdvect( bool use_advect );
  
  // use projection
  void setUseProj( bool use_proj );
  
  // Get/set verbose level
  bool verbose(void) { return VERBOSE; }
  void setVerbose(bool b) { VERBOSE = b; }
  
  void copyState( const StableFluidsSim& otherscene );
  
  // Returns an array containing the horizontal components of the fluid velocity after applying diffusion.
  // Note that the boundary of this array is padded to handle boundary conditions.
  ArrayXs& getHorizontalVelocitiesDiffusion();
  const ArrayXs& getHorizontalVelocitiesDiffusion() const;
  
  // Returns an array containing the vertical components of the fluid velocity after applying diffusion.
  // Note that the boundary of this array is padded to handle boundary conditions.
  ArrayXs& getVerticalVelocitiesDiffusion();
  const ArrayXs& getVerticalVelocitiesDiffusion() const;
  
  // Returns an array containing the horizontal components of the fluid velocity after applying advection.
  // Note that the boundary of this array is padded to handle boundary conditions.
  ArrayXs& getHorizontalVelocitiesAdvect();
  const ArrayXs& getHorizontalVelocitiesAdvect() const;
  
  // Returns an array containing the vertical components of the fluid velocity after applying advection.
  // Note that the boundary of this array is padded to handle boundary conditions.
  ArrayXs& getVerticalVelocitiesAdvect();
  const ArrayXs& getVerticalVelocitiesAdvect() const;
  
  //
  virtual ArrayXb& getHasFluid();
  virtual const ArrayXb& getHasFluid() const;
  
protected:
  // Convenient grid access
  static scalar & d(ArrayXs * d, int i, int j) { return (*d)(i, j); }
  
  static scalar & u(ArrayXs * u, int i, int j) { return (*u)(i, j); }
  static scalar & v(ArrayXs * v, int i, int j) { return (*v)(i, j); }
  
  // Interpolator
  scalar interpolateD(ArrayXs * d, scalar i, scalar j);
  scalar interpolateU(ArrayXs * u, scalar i, scalar j);
  scalar interpolateV(ArrayXs * v, scalar i, scalar j);
  
protected:
  // Time stepping utilities
  virtual void dens_step(int N, ArrayXs * x, ArrayXs * x0, ArrayXs * u, ArrayXs * v, scalar diff, scalar dt);
  virtual void vel_step(int N, ArrayXs * u, ArrayXs * v, ArrayXs * u0, ArrayXs * v0, scalar visc, scalar dt);
  
  virtual void add_source(int N, ArrayXs * x, ArrayXs * x0, scalar dt);
  
  virtual void diffuseD(int N, ArrayXs * x, ArrayXs * x0, scalar diff, scalar dt);
  virtual void diffuseU(int N, ArrayXs * x, ArrayXs * x0, scalar diff, scalar dt);
  virtual void diffuseV(int N, ArrayXs * x, ArrayXs * x0, scalar diff, scalar dt);
  
  virtual void advectD(int N, ArrayXs * x, ArrayXs * x0, ArrayXs * u, ArrayXs * v, scalar dt);
  virtual void advectU(int N, ArrayXs * x, ArrayXs * x0, ArrayXs * u, ArrayXs * v, scalar dt);
  virtual void advectV(int N, ArrayXs * x, ArrayXs * x0, ArrayXs * u, ArrayXs * v, scalar dt);
  
  virtual void project(int N, ArrayXs * u, ArrayXs * v, ArrayXs * u0, ArrayXs * v0);
  
  void SWAP(ArrayXs *& x1, ArrayXs *& x2);
  
protected:
  // Use advection
  bool m_use_advect;
  // Use projection
  bool m_use_project;
  // Diffusion coefficient for tracer particles
  scalar m_diff;
  // Viscosity of fluid
  scalar m_visc;
  // Width of the (square) grid
  int m_N;
  // Density of tracer particles: defined at center of cell
  ArrayXs m_d;
  // Horizontal velocities:       defined at left/right walls of cell
  ArrayXs m_u;
  // Vertical velocities:         defined at top/bottom walls of cell
  ArrayXs m_v;
  
  // For Oracle
  ArrayXs m_uAfterDiffusion;
  ArrayXs m_vAfterDiffusion;
  ArrayXs m_uAfterAdvect;
  ArrayXs m_vAfterAdvect;
  
  bool VERBOSE;
  
  ArrayXb m_all_ones;
};

#endif
