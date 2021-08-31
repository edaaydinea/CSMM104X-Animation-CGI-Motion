#include "ContestDetector.h"
#include <iostream>
#include "TwoDScene.h"
#include <set>
// Given particle positions, computes lists of *potentially* overlapping object
// pairs. How exactly to do this is up to you.
// Inputs: 
//   scene:  The scene object. Get edge information, radii, etc. from here. If 
//           for some reason you'd also like to use particle velocities in your
//           algorithm, you can get them from here too.
//   x:      The positions of the particle.
// Outputs:
//   pppairs: A list of (particle index, particle index) pairs of potentially
//            overlapping particles. IMPORTANT: Each pair should only appear
//            in the list at most once. (1, 2) and (2, 1) count as the same 
//            pair.
//   pepairs: A list of (particle index, edge index) pairs of potential
//            particle-edge overlaps.
//   phpairs: A list of (particle index, halfplane index) pairs of potential
//            particle-halfplane overlaps.


int hash(double minimum, double maximum, double val, int number_of_cells)
{
  int result = (val - minimum)/(maximum - minimum) * number_of_cells;
  return std::max(std::min(number_of_cells - 1, result), 0);
}

void ContestDetector::findCollidingPairs(const TwoDScene &scene, const VectorXs &x, PPList &pppairs, PEList &pepairs, PHList &phpairs)
{
    static int number_of_cells = 0;
    if(number_of_cells == 0)
    {
        number_of_cells = sqrt(scene.getNumParticles());
    }
        
    double minimum_x, maximum_x, minimum_y, maximum_y;
    minimum_x = maximum_x = x[0];
    minimum_y = maximum_y = x[1];

    // particle to Halfplane
    for(int i = 0; i < scene.getNumParticles(); i++)
        {
            if(x[2 * i] > maximum_x)
            {
                maximum_x = x[2*i];
            }
            if (x[2 * i] < minimum_x)
            {
                minimum_x = x[2*i];
            }

            if(x[2 * i+1] > maximum_y)
            {
                maximum_y = x[2*i+1];
            }
            if (x[2 * i+1] < minimum_y)
            {
                minimum_y = x[2*i+1];
            }


            for(int j=0; j<scene.getNumHalfplanes(); j++)
            {
                phpairs.insert(std::pair<int, int>(i,j));
            }

        }

    struct Cell
    {
        std::set<int> vertices;
        std::set<int> edges;
    };

    static Cell *hashgrid = NULL;
    if(!hashgrid)
    {
        hashgrid = new Cell[number_of_cells * number_of_cells];
    }
        
    for(int i = 0; i < number_of_cells * number_of_cells; i++)
        {
            hashgrid[i].vertices.clear();
            hashgrid[i].edges.clear();
        }
  

    for(int i=0; i<scene.getNumParticles(); i++)
    {
        double radius = scene.getRadius(i);
        int px1 = hash(minimum_x, maximum_x, x[2 * i] - radius, number_of_cells);
        int px2 = hash(minimum_x, maximum_x, x[2 * i] + radius, number_of_cells);

        int py1 = hash(minimum_y, maximum_y, x[2 * i+1] - radius, number_of_cells);
        int py2 = hash(minimum_y, maximum_y, x[2 * i+1] + radius, number_of_cells);

        for(int a = px1; a <= px2; a++)
	    {
            for(int b = py1; b <= py2; b++)
            {
                hashgrid[number_of_cells*a + b].vertices.insert(i);
            }
	    }

    }

    // edge + particle
    for(int i=0; i<scene.getNumEdges(); i++)
    {
        double radius = scene.getEdgeRadii()[i];
        int edge1 = scene.getEdge(i).first;
        int edge2 = scene.getEdge(i).second;
        
        int px1 = std::min(hash(minimum_x, maximum_x, x[2 * edge1] - radius, number_of_cells), hash(minimum_x, maximum_x, x[2 * edge2] - radius, number_of_cells));
        int px2 = std::max(hash(minimum_x, maximum_x, x[2 * edge1] + radius, number_of_cells), hash(minimum_x, maximum_x, x[2 * edge2] + radius, number_of_cells));

        int py1 = std::min(hash(minimum_y, maximum_y, x[2 * edge1+1] - radius, number_of_cells), hash(minimum_y, maximum_y, x[2 * edge2+1] - radius, number_of_cells));
        int py2 = std::max(hash(minimum_y, maximum_y, x[2 * edge1+1] + radius, number_of_cells), hash(minimum_y, maximum_y, x[2 * edge2+1] + radius, number_of_cells));

        for(int a = px1; a <= px2; a++)
        {
            for(int b = py1; b <= py2; b++)
            {
                hashgrid[number_of_cells*a + b].edges.insert(i);
            }
        }
    }

    for(int i=0; i<number_of_cells*number_of_cells; i++)
    {
        for(std::set<int>::iterator iterator1 = hashgrid[i].vertices.begin(); iterator1 != hashgrid[i].vertices.end(); ++iterator1)
        {
            std::set<int>::iterator interator2 = iterator1;
            for(++interator2; interator2 != hashgrid[i].vertices.end(); ++interator2)
            {
                // particle to particle
                pppairs.insert(std::pair<int, int>(*iterator1, *interator2));
            }
            for(std::set<int>::iterator interator2 = hashgrid[i].edges.begin(); interator2 != hashgrid[i].edges.end(); ++interator2)
            {
                // particle to edge
                pepairs.insert(std::pair<int, int>(*iterator1, *interator2));
            }
        }
    }
      
}