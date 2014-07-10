#include <iostream>
#include <sys/types.h>
#include <time.h>
//#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <cfloat>
#include <mpfr.h>

class NavigationController
{
public:

    NavigationController();


    static void fcn_A_B(int numOfRobots, int numOfParts, double partDist, mpfr_t A, mpfr_t B, double b_[][4], double bt_[][3],  double brs_[][4], double bp[][4], double ro,int rID);

    static void fcn_dAdx(mpfr_t dAdx, mpfr_rnd_t MPFR_RND, double b_[][4], double bt_[][3], int rID);

    static void fcn_dAdy(mpfr_t dAdy, mpfr_rnd_t MPFR_RND, double b_[][4], double bt_[][3], int rID);

    static void fcn_dBdx(int numOfRobots, int numOfParts, double partDist, mpfr_t dBdx, mpfr_t B, mpfr_rnd_t MPFR_RND, double b_[][4], double brs_[][4], double bp[][4], double ro, int rID);

    static void fcn_dBdy(int numOfRobots, int numOfParts, double partDist, mpfr_t dBdy, mpfr_t B, mpfr_rnd_t MPFR_RND, double b_[][4], double brs_[][4], double bp[][4], double ro, int rID);

    static void fcn_dFdx(mpfr_t dFdx, mpfr_t A, mpfr_t B, int kk, int kq, mpfr_t dAdx, mpfr_t dBdx, mpfr_rnd_t MPFR_RND);

    static void fcn_dFdy(mpfr_t dFdy, mpfr_t A, mpfr_t B, int kk, int kq, mpfr_t dAdy, mpfr_t dBdy, mpfr_rnd_t MPFR_RND);

    static void robotContoller(double bout[], int numOfRobots, int numOfParts, double partDist, double bin_[][4], double bt_[][3], double b_rs_[][4], double bp[][4], double ro, double kkLimits[], int rID);




};
