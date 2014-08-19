#include <iostream>
#include <sys/types.h>
#include <time.h>
//#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <cfloat>
#include <mpfr.h>
#include <vector>

class NavigationController
{
public:

    NavigationController();


    static void fcn_A_B(int numOfRobots, int numOfParts, double partDist, mpfr_t A, mpfr_t B, std::vector<std::vector<double> > b_, std::vector<std::vector<double> > bt_,  std::vector<std::vector<double> > brs_, std::vector<std::vector<double> > bp, double ro,int rID);

    static void fcn_dAdx(mpfr_t dAdx, mpfr_rnd_t MPFR_RND, std::vector<std::vector<double> > b_, std::vector<std::vector<double> > bt_, int rID);

    static void fcn_dAdy(mpfr_t dAdy, mpfr_rnd_t MPFR_RND, std::vector<std::vector<double> > b_, std::vector<std::vector<double> > bt_, int rID);

    static void fcn_dBdx(int numOfRobots, int numOfParts, double partDist, mpfr_t dBdx, mpfr_t B, mpfr_rnd_t MPFR_RND, std::vector<std::vector<double> > b_, std::vector<std::vector<double> > brs_, std::vector<std::vector<double> > bp, double ro, int rID);

    static void fcn_dBdy(int numOfRobots, int numOfParts, double partDist, mpfr_t dBdy, mpfr_t B, mpfr_rnd_t MPFR_RND, std::vector<std::vector<double> > b_, std::vector<std::vector<double> > brs_, std::vector<std::vector<double> > bp, double ro, int rID);

    static void fcn_dFdx(mpfr_t dFdx, mpfr_t dQdx, mpfr_t A, mpfr_t B, int kk, int kq, mpfr_t dAdx, mpfr_t dBdx, mpfr_rnd_t MPFR_RND);

    static void fcn_dFdy(mpfr_t dFdy, mpfr_t dQdy, mpfr_t A, mpfr_t B, int kk, int kq, mpfr_t dAdy, mpfr_t dBdy, mpfr_rnd_t MPFR_RND);

    static void robotContoller(double bout[],double* boutLengthOfVel, int numOfRobots, int numOfParts, double partDist, std::vector<std::vector<double> > bin_, std::vector<std::vector<double> > bt_, std::vector<std::vector<double> > b_rs_, std::vector<std::vector<double> > bp, double ro, double kkLimits[], int rID);




};
