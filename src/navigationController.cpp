#include "navigationController.h"



// Alpha, Beta
void NavigationController::fcn_A_B(int numOfRobots, int numOfParts, double partDist, mpfr_t A, mpfr_t B, double b_[][4], double bt_[][3],  double brs_[][4], double bp[][4], double ro, int rID)
{
    //mpfr_t A, B;
    mpfr_t temp;
    mpfr_rnd_t MPFR_RND;
    MPFR_RND = mpfr_get_default_rounding_mode();

    mpfr_init (temp);
    //mpfr_init (A);
    //mpfr_init (B);

    //double AA=0;

    mpfr_set_d(A, 0.0, MPFR_RND);

    for(int i=1;i<=numOfRobots;i++)
    {
        if (b_[i][3]!=0)
        {
            double dist = pow((b_[i][1] - bt_[i][1]),2) + pow((b_[i][2] - bt_[i][2]),2);
            mpfr_set_d(temp, dist, MPFR_RND);
            mpfr_add(A, A, temp, MPFR_RND);
            //AA = AA + dist;
        }
    }


    /*
    mpfr_out_str (stdout, 10, 0, A, MPFR_RND);
    printf("\n");
    */
    mpfr_set_d(B, 1.0, MPFR_RND);

    for(int i=1;i<=numOfRobots;i++)
    {
        if (i!=rID)
        {
            double  dist=-1;

            if (b_[i][3]!=0)
            {
                dist = abs(pow((b_[rID][1] - b_[i][1]),2) + pow((b_[rID][2] - b_[i][2]),2) -pow((b_[rID][3]+b_[i][3]),2));
                /*
                if (dist<0)
                {
                    int dedede=1;
                }
                */
            }
            else if (brs_[i][3]!=0)
            {
                dist = abs(pow((b_[rID][1] - brs_[i][1]),2) + pow((b_[rID][2] - brs_[i][2]),2) -pow((b_[rID][3]+brs_[i][3]),2));
            }

            if (dist>0)
            {
                mpfr_set_d(temp, dist, MPFR_RND);
                mpfr_mul(B, B, temp, MPFR_RND);
            }
        }
    }


    // others
    for(int i=1;i<=numOfRobots;i++)
    {
        for(int j=(i+1);j<=numOfRobots;j++)
        {
            if ( (i!=rID) && (j!=rID))
            {
                double dist=-1;

                if ((b_[i][3]!=0) && (b_[j][3]!=0))
                {
                    dist = abs( pow((b_[i][1] - b_[j][1]),2) + pow((b_[i][2] - b_[j][2]),2) -pow((b_[i][3]+b_[j][3]),2));
                    /*
                    if (dist<0)
                    {
                        int dedede=1;
                    }
                    */
                }
                else if ((brs_[i][3]!=0) && (b_[j][3]!=0))
                {
                    dist =abs( pow((brs_[i][1] - b_[j][1]),2) + pow((brs_[i][2] - b_[j][2]),2) -pow((brs_[i][3]+b_[j][3]),2));
                }
                else if ((b_[i][3]!=0) && (brs_[j][3]!=0))
                {
                    dist =abs( pow((b_[i][1] - brs_[j][1]),2) + pow((b_[i][2] - brs_[j][2]),2) -pow((b_[i][3]+brs_[j][3]),2));
                }
                else if ((brs_[i][3]!=0) && (brs_[j][3]!=0))
                {
                    dist =abs( pow((brs_[i][1] - brs_[j][1]),2) + pow((brs_[i][2] - brs_[j][2]),2) -pow((brs_[i][3]+brs_[j][3]),2));
                }

                if (dist>0)
                {
                    mpfr_set_d(temp, dist, MPFR_RND);
                    mpfr_mul(B, B, temp, MPFR_RND);
                }
            }
        }
    }

    //parts
    for(int j=1;j<=numOfParts;j++)
    {
        double dist = abs(pow((b_[rID][1] - bp[j][1]),2) + pow((b_[rID][2] - bp[j][2]),2) -pow((b_[rID][3]+bp[j][3]),2));
        if (dist<partDist*partDist)
        {
            mpfr_set_d(temp, dist, MPFR_RND);
            mpfr_mul(B, B, temp, MPFR_RND);
        }
    }


    // outer - bounded
    for(int i=1;i<=numOfRobots;i++)
     {

        double dist=-1;
        if (b_[i][3]!=0)
        {
            dist = abs((ro - b_[i][3])*(ro - b_[i][3]) - b_[i][1]*b_[i][1] - b_[i][2]*b_[i][2]);
            if (dist<0)
            {
                int dedede=1;
            }
        }
        else if (brs_[i][3]!=0)
        {
            dist = abs((ro - brs_[i][3])*(ro - brs_[i][3]) - brs_[i][1]*brs_[i][1] - brs_[i][2]*brs_[i][2]);
        }

        if (dist>0)
        {
            mpfr_set_d(temp, dist, MPFR_RND);
            mpfr_mul(B, B, temp, MPFR_RND);
        }
     }


     mpfr_clear (temp);
}



void NavigationController::fcn_dAdx(mpfr_t dAdx, mpfr_rnd_t MPFR_RND, double b_[][4], double bt_[][3], int rID)
{
    mpfr_set_d(dAdx, 2*(b_[rID][1]-bt_[rID][1]), MPFR_RND);
}


void NavigationController::fcn_dAdy(mpfr_t dAdy, mpfr_rnd_t MPFR_RND, double b_[][4], double bt_[][3], int rID)
{
    mpfr_set_d(dAdy, 2*(b_[rID][2]-bt_[rID][2]), MPFR_RND);
}


void NavigationController::fcn_dBdx(int numOfRobots, int numOfParts, double partDist, mpfr_t dBdx, mpfr_t B, mpfr_rnd_t MPFR_RND, double b_[][4], double brs_[][4], double bp[][4], double ro, int rID)
{
    mpfr_t temp;
    mpfr_init (temp);

    mpfr_set_d(dBdx, 0.0, MPFR_RND);
    for(int i=1;i<=numOfRobots;i++)
    {
        if (i!=rID)
        {
            double val;
            if (b_[i][3]!=0)
            {
                val = 2*(b_[rID][1]-b_[i][1])/( pow((b_[rID][1] - b_[i][1]),2) + pow((b_[rID][2] - b_[i][2]),2) - pow((b_[rID][3]+b_[i][3]),2) );
                mpfr_set_d(temp, val, MPFR_RND);
                mpfr_add(dBdx, dBdx, temp, MPFR_RND);

            }
            else if (brs_[i][3]!=0)
            {
                val = 2*(b_[rID][1]-brs_[i][1])/( pow((b_[rID][1] - brs_[i][1]),2) + pow((b_[rID][2] - brs_[i][2]),2) - pow((b_[rID][3]+brs_[i][3]),2) );
                mpfr_set_d(temp, val, MPFR_RND);
                mpfr_add(dBdx, dBdx, temp, MPFR_RND);
            }
        }
    }

    double val = (-2*b_[rID][1])/ (pow((ro - b_[rID][3]),2) - b_[rID][1]*b_[rID][1] - b_[rID][2]*b_[rID][2] ) ; // bounded
    mpfr_set_d(temp, val, MPFR_RND);
    mpfr_add(dBdx, dBdx, temp, MPFR_RND);


    for(int i=1;i<=numOfParts;i++)
    {
        double dist = fabs(pow((b_[rID][1] - bp[i][1]),2) + pow((b_[rID][2] - bp[i][2]),2) -pow((b_[rID][3]+bp[i][3]),2));
        if (dist<partDist*partDist)
        {
            double val = 2*(b_[rID][1]-bp[i][1])/( pow((b_[rID][1] - bp[i][1]),2) + pow((b_[rID][2] - bp[i][2]),2) - pow((b_[rID][3]+bp[i][3]),2) );
            mpfr_set_d(temp, val, MPFR_RND);
            mpfr_add(dBdx, dBdx, temp, MPFR_RND);
        }
    }

    mpfr_mul(dBdx, dBdx, B, MPFR_RND);

    mpfr_clear (temp);
}

void NavigationController::fcn_dBdy(int numOfRobots, int numOfParts, double partDist, mpfr_t dBdy, mpfr_t B, mpfr_rnd_t MPFR_RND, double b_[][4], double brs_[][4], double bp[][4], double ro, int rID)
{
    mpfr_t temp;
    mpfr_init (temp);

    mpfr_set_d(dBdy, 0.0, MPFR_RND);
    for(int i=1;i<=numOfRobots;i++)
    {
        if (i!=rID)
        {
            double val;
            if (b_[i][3]!=0)
            {
                val = 2*(b_[rID][2]-b_[i][2])/( pow((b_[rID][1] - b_[i][1]),2) + pow((b_[rID][2] - b_[i][2]),2) - pow((b_[rID][3]+b_[i][3]),2) );
                mpfr_set_d(temp, val, MPFR_RND);
                mpfr_add(dBdy, dBdy, temp, MPFR_RND);
            }
            else if (brs_[i][3]!=0)
            {
                val = 2*(b_[rID][2]-brs_[i][2])/( pow((b_[rID][1] - brs_[i][1]),2) + pow((b_[rID][2] - brs_[i][2]),2) - pow((b_[rID][3]+brs_[i][3]),2) );
                mpfr_set_d(temp, val, MPFR_RND);
                mpfr_add(dBdy, dBdy, temp, MPFR_RND);
            }
        }
    }
    double val = (-2*b_[rID][2])/( pow((ro - b_[rID][3]),2) - b_[rID][1]*b_[rID][1] - b_[rID][2]*b_[rID][2] ) ; // bounded
    mpfr_set_d(temp, val, MPFR_RND);
    mpfr_add(dBdy, dBdy, temp, MPFR_RND);


    for(int i=1;i<=numOfParts;i++)
    {
        double dist = fabs(pow((b_[rID][1] - bp[i][1]),2) + pow((b_[rID][2] - bp[i][2]),2) -pow((b_[rID][3]+bp[i][3]),2));
        if (dist<partDist*partDist)
        {
            double val = 2*(b_[rID][2]-bp[i][2])/( pow((b_[rID][1] - bp[i][1]),2) + pow((b_[rID][2] - bp[i][2]),2) - pow((b_[rID][3]+bp[i][3]),2) );
            mpfr_set_d(temp, val, MPFR_RND);
            mpfr_add(dBdy, dBdy, temp, MPFR_RND);
        }
    }

    mpfr_mul(dBdy, dBdy, B, MPFR_RND);

    mpfr_clear (temp);
}


void NavigationController::fcn_dFdx(mpfr_t dFdx, mpfr_t A, mpfr_t B, int kk, int kq, mpfr_t dAdx, mpfr_t dBdx, mpfr_rnd_t MPFR_RND)
{

    mpfr_t gammak, gammakB, temp, temp1, temp2, temp3, temp4, one, two, kk_m, kq_m, minusOne;

    mpfr_init (gammak);
    mpfr_init (gammakB);
    mpfr_init (temp);
    mpfr_init (temp1);
    mpfr_init (temp2);
    mpfr_init (temp3);
    mpfr_init (temp4);
    mpfr_init (one);
    mpfr_init (two);
    mpfr_init (minusOne);
    mpfr_init (kk_m);
    mpfr_init (kq_m);

    mpfr_set_d(one, 1.0, MPFR_RND);
    mpfr_set_d(two, 2.0, MPFR_RND);
    mpfr_set_d(minusOne, -1.0, MPFR_RND);
    mpfr_set_d(kk_m, kk, MPFR_RND);
    mpfr_set_d(kq_m, kq, MPFR_RND);

    /////  new ///
    mpfr_t Q, dQdA, dQdB, dQdx;
    mpfr_init (Q);
    mpfr_init (dQdA);
    mpfr_init (dQdB);
    mpfr_init (dQdx);


    // Q= Math.pow(A,Ak)/B;
    mpfr_pow(Q, A, kk_m, MPFR_RND);
    mpfr_div(Q, Q, B, MPFR_RND);

    // dQdA = Ak*Math.pow(A,Ak-1)/B;
    mpfr_set_d(temp, kk-1, MPFR_RND);
    mpfr_pow(dQdA, A, temp, MPFR_RND);
    mpfr_mul(dQdA, dQdA, kk_m, MPFR_RND);
    mpfr_div(dQdA, dQdA, B, MPFR_RND);

    // dQdB = -1.0*Math.pow(A,Ak)/(B*B);
    mpfr_div(dQdB, Q, B, MPFR_RND);
    mpfr_mul(dQdB, dQdB, minusOne, MPFR_RND);

    // dAdx = dAdx();
    // dBdx = dBdx();

    // dQdx = dQdA*dAdx+dQdB*dBdx;
    mpfr_mul(temp1, dQdA, dAdx, MPFR_RND);
    mpfr_mul(temp2, dQdB, dBdx, MPFR_RND);
    mpfr_add(dQdx, temp1, temp2, MPFR_RND);

    // dFdx = (1/kq)*(Q/(Q+1))^((1/kq)-1)*(Q+1)^(-2)*dQdx;
    mpfr_set_d(temp, ((1.0/kq)-1.0), MPFR_RND);
    mpfr_add(temp1, Q, one, MPFR_RND);
    mpfr_div(temp1, Q, temp1, MPFR_RND);
    mpfr_pow(temp1, temp1, temp, MPFR_RND);

    mpfr_set_d(temp, -2.0, MPFR_RND);
    mpfr_add(temp2, Q, one, MPFR_RND);
    mpfr_pow(temp2, temp2, temp, MPFR_RND);

    mpfr_mul(dFdx, temp1, temp2, MPFR_RND);
    mpfr_mul(dFdx, dFdx, dQdx, MPFR_RND);
    mpfr_div(dFdx, dFdx, kq_m, MPFR_RND);

/*
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dQdA, MPFR_RND);
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dQdB, MPFR_RND);
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dFdx, MPFR_RND);
    printf("\n");
*/

/*
    //gammak = mpower(A,kk);
    //gammakB = mpower((gammak+B),(1/kk));

    mpfr_pow(gammak, A, kk_m, MPFR_RND);
    mpfr_add(gammak, gammak, B, MPFR_RND);
    mpfr_div(temp, one, kk_m, MPFR_RND);
    mpfr_pow(gammakB, gammak, temp, MPFR_RND);

    //dFdx =  (1/mpower(gammakB,2))* ( gammakB * dAdx - (A/kk)*mpower(gammakB,(1-kk)) * (kk*mpower(A,(kk-1))*dAdx+ dBdx) );

    //temp1 = (1/mpower(gammakB,2))
    mpfr_pow(temp, gammakB, two, MPFR_RND);
    mpfr_div(temp1, one, temp, MPFR_RND);

    //temp2 = gammakB * dAdx
    mpfr_mul(temp2, gammakB, dAdx, MPFR_RND);

    //temp3 = (A/kk)*mpower(gammakB,(1-kk))
    mpfr_div(temp3, A, kk_m, MPFR_RND);
    mpfr_set_d(temp, 1-kk, MPFR_RND);
    mpfr_pow(temp, gammakB, temp, MPFR_RND);
    mpfr_mul(temp3, temp3, temp, MPFR_RND);

    // temp4 = (kk*mpower(A,(kk-1))*dAdx+ dBdx)
    mpfr_set_d(temp, kk-1, MPFR_RND);
    mpfr_pow(temp, A, temp, MPFR_RND);
    mpfr_mul(temp4, kk_m,  temp, MPFR_RND);
    mpfr_mul(temp4, temp4, dAdx, MPFR_RND);
    mpfr_add(temp4, temp4, dBdx, MPFR_RND);


    // dFdx = temp1 * ( temp2 - temp3*temp4)
    mpfr_mul(temp, temp3, temp4, MPFR_RND);
    mpfr_sub(temp, temp2, temp, MPFR_RND);
    mpfr_mul(dFdx, temp1, temp, MPFR_RND);

    printf("\n");
    mpfr_out_str (stdout, 10, 0, dFdx, MPFR_RND);
    printf("\n");
*/
    mpfr_clear (gammak);
    mpfr_clear (gammakB);
    mpfr_clear (temp);
    mpfr_clear (temp1);
    mpfr_clear (temp2);
    mpfr_clear (temp3);
    mpfr_clear (temp4);
    mpfr_clear (one);
    mpfr_clear (minusOne);
    mpfr_clear (two);
    mpfr_clear (kk_m);
    mpfr_clear (kq_m);
    mpfr_clear (Q);
    mpfr_clear (dQdA);
    mpfr_clear (dQdB);
    mpfr_clear (dQdx);
}


void NavigationController::fcn_dFdy(mpfr_t dFdy, mpfr_t A, mpfr_t B, int kk, int kq, mpfr_t dAdy, mpfr_t dBdy, mpfr_rnd_t MPFR_RND)
{
//dFdy = (1/kq)*(Q/(Q+1))^((1/kq)-1)*(Q+1)^(-2)*dQdy;

  //gammak = A^kk;
  //gammakB = (gammak+B)^(1/kk);
  //dFdy = (1/gammakB^2)*(gammakB*dAdy - (A/kk)*gammakB^(1-kk)*(kk*A^(kk-1)*dAdy+ dBdy));

    mpfr_t gammak, gammakB, temp, temp1, temp2, temp3, temp4, one, two, kk_m, kq_m, minusOne;

    mpfr_init (gammak);
    mpfr_init (gammakB);
    mpfr_init (temp);
    mpfr_init (temp1);
    mpfr_init (temp2);
    mpfr_init (temp3);
    mpfr_init (temp4);
    mpfr_init (one);
    mpfr_init (two);
    mpfr_init (minusOne);
    mpfr_init (kk_m);
    mpfr_init (kq_m);

    mpfr_set_d(one, 1.0, MPFR_RND);
    mpfr_set_d(two, 2.0, MPFR_RND);
    mpfr_set_d(minusOne, -1.0, MPFR_RND);
    mpfr_set_d(kk_m, kk, MPFR_RND);
    mpfr_set_d(kq_m, kq, MPFR_RND);


    /////  new ///
    mpfr_t Q, dQdA, dQdB, dQdy;
    mpfr_init (Q);
    mpfr_init (dQdA);
    mpfr_init (dQdB);
    mpfr_init (dQdy);


    // Q= Math.pow(A,Ak)/B;
    mpfr_pow(Q, A, kk_m, MPFR_RND);
    mpfr_div(Q, Q, B, MPFR_RND);

    // dQdA = Ak*Math.pow(A,Ak-1)/B;
    mpfr_set_d(temp, kk-1, MPFR_RND);
    mpfr_pow(dQdA, A, temp, MPFR_RND);
    mpfr_mul(dQdA, dQdA, kk_m, MPFR_RND);
    mpfr_div(dQdA, dQdA, B, MPFR_RND);

    // dQdB = -1.0*Math.pow(A,Ak)/(B*B);
    mpfr_div(dQdB, Q, B, MPFR_RND);
    mpfr_mul(dQdB, dQdB, minusOne, MPFR_RND);

    // dAdy = dAdx();
    // dBdy = dBdx();

    // dQdy = dQdA*dAdy+dQdB*dBdy;
    mpfr_mul(temp1, dQdA, dAdy, MPFR_RND);
    mpfr_mul(temp2, dQdB, dBdy, MPFR_RND);
    mpfr_add(dQdy, temp1, temp2, MPFR_RND);

    // dFdy = (1/kq)*(Q/(Q+1))^((1/kq)-1)*(Q+1)^(-2)*dQdy;
    mpfr_set_d(temp, ((1.0/kq)-1.0), MPFR_RND);
    mpfr_add(temp1, Q, one, MPFR_RND);
    mpfr_div(temp1, Q, temp1, MPFR_RND);
    mpfr_pow(temp1, temp1, temp, MPFR_RND);

    mpfr_set_d(temp, -2.0, MPFR_RND);
    mpfr_add(temp2, Q, one, MPFR_RND);
    mpfr_pow(temp2, temp2, temp, MPFR_RND);

    mpfr_mul(dFdy, temp1, temp2, MPFR_RND);
    mpfr_mul(dFdy, dFdy, dQdy, MPFR_RND);
    mpfr_div(dFdy, dFdy, kq_m, MPFR_RND);

/*
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dQdA, MPFR_RND);
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dQdB, MPFR_RND);
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dFdy, MPFR_RND);
    printf("\n");
*/


/*
    //gammak = mpower(A,kk);
    //gammakB = mpower((gammak+B),(1/kk));

    mpfr_pow(gammak, A, kk_m, MPFR_RND);
    mpfr_add(gammak, gammak, B, MPFR_RND);
    mpfr_div(temp, one, kk_m, MPFR_RND);
    mpfr_pow(gammakB, gammak, temp, MPFR_RND);

    //dFdx =  (1/mpower(gammakB,2))* ( gammakB * dAdx - (A/kk)*mpower(gammakB,(1-kk)) * (kk*mpower(A,(kk-1))*dAdx+ dBdx) );

    //temp1 = (1/mpower(gammakB,2))
    mpfr_pow(temp, gammakB, two, MPFR_RND);
    mpfr_div(temp1, one, temp, MPFR_RND);

    //temp2 = gammakB * dAdx
    mpfr_mul(temp2, gammakB, dAdy, MPFR_RND);

    //temp3 = (A/kk)*mpower(gammakB,(1-kk))
    mpfr_div(temp3, A, kk_m, MPFR_RND);
    mpfr_set_d(temp, 1-kk, MPFR_RND);
    mpfr_pow(temp, gammakB, temp, MPFR_RND);
    mpfr_mul(temp3, temp3, temp, MPFR_RND);

    // temp4 = (kk*mpower(A,(kk-1))*dAdx+ dBdx)
    mpfr_set_d(temp, kk-1, MPFR_RND);
    mpfr_pow(temp, A, temp, MPFR_RND);
    mpfr_mul(temp4, kk_m,  temp, MPFR_RND);
    mpfr_mul(temp4, temp4, dAdy, MPFR_RND);
    mpfr_add(temp4, temp4, dBdy, MPFR_RND);


    // dFdx = temp1 * ( temp2 - temp3*temp4)
    mpfr_mul(temp, temp3, temp4, MPFR_RND);
    mpfr_sub(temp, temp2, temp, MPFR_RND);
    mpfr_mul(dFdy, temp1, temp, MPFR_RND);

    printf("\n");
    mpfr_out_str (stdout, 10, 0, dFdy, MPFR_RND);
    printf("\n");
*/

    mpfr_clear (gammak);
    mpfr_clear (gammakB);
    mpfr_clear (temp);
    mpfr_clear (temp1);
    mpfr_clear (temp2);
    mpfr_clear (temp3);
    mpfr_clear (temp4);
    mpfr_clear (one);
    mpfr_clear (minusOne);
    mpfr_clear (two);
    mpfr_clear (kk_m);
    mpfr_clear (kq_m);
    mpfr_clear (Q);
    mpfr_clear (dQdA);
    mpfr_clear (dQdB);
    mpfr_clear (dQdy);
}


void NavigationController::robotContoller(double bout[], int numOfRobots, int numOfParts, double partDist, double bin_[][4], double bt_[][3], double b_rs_[][4], double bp[][4], double ro, double kkLimits[], int rID)
{
    mpfr_t A, B, dBdx, dBdy, dAdx, dAdy, dFdx, dFdy, norm, temp1, temp2,minusOne, two;
    mpfr_t logA_m, logB_m;// divlogBlogA;
    mpfr_rnd_t MPFR_RND;
    MPFR_RND = mpfr_get_default_rounding_mode();

    int kk, kq;
    double logA, logB;

    //mpfr_set_default_prec(53);

    mpfr_init (A);
    mpfr_init (B);
    mpfr_init (logA_m);
    mpfr_init (logB_m);
    mpfr_init (dAdx);
    mpfr_init (dAdy);
    mpfr_init (dBdx);
    mpfr_init (dBdy);
    mpfr_init (dFdx);
    mpfr_init (dFdy);
    mpfr_init (norm);
    mpfr_init (temp1);
    mpfr_init (temp2);
    mpfr_init (minusOne);
    mpfr_init (two);


    fcn_A_B(numOfRobots, numOfParts, partDist, A, B, bin_, bt_, b_rs_, bp, ro, rID);

    /*
    printf("\n");
    mpfr_out_str (stdout, 10, 0, A, MPFR_RND);
    printf("\n");
    mpfr_out_str (stdout, 10, 0, B, MPFR_RND);
    printf("\n");
*/

    mpfr_log10(logA_m, A, MPFR_RND);
    mpfr_log10(logB_m, B, MPFR_RND);
    logA = mpfr_get_d(logA_m, MPFR_RND);
    logB = mpfr_get_d(logB_m, MPFR_RND);
    //mpfr_div(divlogBlogA, logB, logA, MPFR_RND);
    //kk = ceil(mpfr_get_d(divlogBlogA, MPFR_RND));
    kk = ceil(logB/logA);
    kk = abs(kk + (kk%2));

    /*
    if (kk<0)
    {
        printf("\n");
        mpfr_out_str (stdout, 10, 0, A, MPFR_RND);
        printf("\n");
        mpfr_out_str (stdout, 10, 0, B, MPFR_RND);
        printf("\n");
        mpfr_out_str (stdout, 10, 0, logA_m, MPFR_RND);
        printf("\n");
        mpfr_out_str (stdout, 10, 0, logB_m, MPFR_RND);
        printf("\n");
    }
    */

    if (kk<kkLimits[0])
    {
        kk=kkLimits[0];
    }
    else
    {
        if (kk>kkLimits[1])
            kk=kkLimits[1];
    }

/*
    if ( (kk<(numOfRobots + numOfRobots*(numOfRobots-1)/2)))
    {
        int temp;
        temp = numOfRobots + numOfRobots*(numOfRobots-1)/2;
        kk = abs(temp + (temp%2));
    }

*/
    kq=kk;

    fcn_dAdx(dAdx, MPFR_RND, bin_, bt_, rID);
    fcn_dBdx(numOfRobots, numOfParts, partDist, dBdx, B, MPFR_RND, bin_, b_rs_, bp, ro, rID);
    fcn_dAdy(dAdy, MPFR_RND, bin_, bt_, rID);
    fcn_dBdy(numOfRobots, numOfParts, partDist, dBdy, B, MPFR_RND, bin_, b_rs_, bp, ro, rID);


    fcn_dFdx(dFdx, A, B, kk, kq, dAdx, dBdx, MPFR_RND);
    fcn_dFdy(dFdy, A, B, kk, kq, dAdy, dBdy, MPFR_RND);
/*
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dFdx, MPFR_RND);
    printf("\n");
    mpfr_out_str (stdout, 10, 0, dFdy, MPFR_RND);
    printf("\n");
*/


    mpfr_set_d(two, 2.0, MPFR_RND);
    mpfr_pow(temp1, dFdx, two, MPFR_RND);
    mpfr_pow(temp2, dFdy, two, MPFR_RND);

    mpfr_add(norm, temp1, temp2, MPFR_RND);
    mpfr_sqrt(norm, norm, MPFR_RND);

    mpfr_div(dFdx, dFdx, norm, MPFR_RND);
    mpfr_div(dFdy, dFdy, norm, MPFR_RND);

    bout[0] = -1.0*mpfr_get_d(dFdx, MPFR_RND);
    bout[1] = -1.0*mpfr_get_d(dFdy, MPFR_RND);

    mpfr_clear (A);
    mpfr_clear (B);
    mpfr_clear (logA_m);
    mpfr_clear (logB_m);
    mpfr_clear (dAdx);
    mpfr_clear (dAdy);
    mpfr_clear (dBdx);
    mpfr_clear (dBdy);
    mpfr_clear (dFdx);
    mpfr_clear (dFdy);
    mpfr_clear (norm);
    mpfr_clear (temp1);
    mpfr_clear (temp2);
    mpfr_clear (minusOne);
    mpfr_clear (two);

}
