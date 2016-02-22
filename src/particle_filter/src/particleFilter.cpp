//#include <string.h>
#include<iostream>
#include <random>
#include<cmath>
using namespace std;	
#include "particleFilter.h"

# define Pi          3.141592653589793238462643383279502884L

#define SQ(x) ((x)*(x))

particleFilter::particleFilter (int n_particles,
				double Xstd_ob, double Xstd_tran,
				double Xstd_scatter, double R)
  : N(n_particles), Xstd_ob(Xstd_ob), Xstd_tran(Xstd_tran),
    Xstd_scatter(Xstd_scatter), R(R), firstObs(true)
{
  bzero(b_Xprior, 2*sizeof(cspace));
  bzero(b_Xpre, 2*sizeof(cspace));

  X = new cspace[N];
  bzero(X, N*sizeof(cspace));
  X0 = new cspace[N];
  bzero(X0, N*sizeof(cspace));

  Xpre_weight = new cspace[N];
  bzero(Xpre_weight, N*sizeof(cspace));
  W = new double[N];
  bzero(W, N*sizeof(double));
}

void particleFilter::setDistribution (cspace dist[2])
{
  memcpy(b_Xprior, dist, 2*sizeof(cspace));
  memcpy(b_Xpre,   dist, 2*sizeof(cspace));
  memcpy(X_est,    dist, 2*sizeof(cspace));
}

void particleFilter::getAllParticles(cspace *particles)
{
  for(int i=0; i<N; i++){
    for(int j=0; j<cdim; j++){
      particles[i][j] = X[i][j];
    }
  }
}

void particleFilter::addObservation (double obs[3])
{
  std::default_random_engine generator;
  normal_distribution<double> dist2(0, Xstd_scatter);

  if (!firstObs) {
    bzero(b_Xpre, 2*sizeof(cspace));
    for (int j = 0; j < N; j++) {
      X0[j][0] += dist2(generator);
      X0[j][1] += dist2(generator);
      X0[j][2] += dist2(generator);
      b_Xpre[0][0] += X0[j][0] / N;
      b_Xpre[0][1] += X0[j][1] / N;
      b_Xpre[0][2] += X0[j][2] / N;
    }
    for (int j = 0; j < N; j++)
      {
	b_Xpre[1][0] += SQ(X0[j][0] - b_Xpre[0][0]) / N;
	b_Xpre[1][1] += SQ(X0[j][1] - b_Xpre[0][1]) / N;
	b_Xpre[1][2] += SQ(X0[j][2] - b_Xpre[0][2]) / N;
      }
    b_Xpre[1][0] = sqrt(b_Xpre[1][0]);
    b_Xpre[1][1] = sqrt(b_Xpre[1][1]);
    b_Xpre[1][2] = sqrt(b_Xpre[1][2]);
  }
  bool iffar = update_particles(X, b_Xprior, b_Xpre, Xstd_ob, R, obs, N);
  if (firstObs)
    {
      firstObs = false;
      memcpy(X0, X, N*sizeof(cspace));
      memcpy(Xpre_weight, X, N*sizeof(cspace));
    }
  else if (iffar == true)
    {
      memcpy(X0, Xpre_weight, N*sizeof(cspace));
    }
  calc_weight(W, N, Xstd_tran, X0, X);
  memcpy(Xpre_weight, X0, N*sizeof(cspace));
  resample_particles(X0, X, W, N);
  X_est[0][0] = 0;
  X_est[0][1] = 0;
  X_est[0][2] = 0;
  X_est[1][0] = 0;
  for (int j = 0; j < N; j++)
    {
      X_est[0][0] += X0[j][0] / N;
      X_est[0][1] += X0[j][1] / N;
      X_est[0][2] += X0[j][2] / N;
    }
  for (int j = 0; j < N; j++)
    {
      X_est[1][0] += (SQ(X0[j][0] - X_est[0][0]) + SQ(X0[j][1] - X_est[0][1]) +
		      SQ(X0[j][2] - X_est[0][2])) / N;
    }
  X_est[1][0] = sqrt(X_est[1][0]);

  if (X_est[1][0] < 0.005 && (abs(X_est[0][0] - b_Xpre[0][0])>0.001 ||
			      abs(X_est[0][1] - b_Xpre[0][1])>0.001 ||
			      abs(X_est[0][2] - b_Xpre[0][2])>0.001 ))
    Xstd_scatter = 0.01;
  else
    Xstd_scatter = 0.0001;
}

void particleFilter::create_particles(cspace *X, cspace b_Xprior[2],
				      int n_particles)
{
  random_device rd;

  mt19937 e2(rd());

  normal_distribution<double> dist(0, 1);
  for (int i = 0; i < n_particles;i++)
    {
      X[i][0] = b_Xprior[0][0] + b_Xprior[1][0] * (dist(e2));
      X[i][1] = b_Xprior[0][1] + b_Xprior[1][1] * (dist(e2));
      X[i][2] = b_Xprior[0][2] + b_Xprior[1][2] * (dist(e2));
    }
};

bool particleFilter::update_particles(cspace *X, cspace b_Xprior[2],
				      cspace b_Xpre[2], double Xstd_ob,
				      double R, double cur_M[3],
				      int n_particles)
{
  random_device rd;

  mt19937 e2(rd());
  normal_distribution<double> dist(0, 1);
  int i = 0;
  int count = 0;
  int factor = 0;
  int nIter = n_particles*3000;
  bool iffar = false;
  cspace *b_X = b_Xpre;
  double tempa, tempb, tempc, D;
  while (i < n_particles)
    {
      if ((count == nIter && iffar==false) || (i>0 && count/i>3000))
	{
	  iffar = true;
	  b_X = b_Xprior;
	  count = 0;
	}
      tempa = b_X[0][0] + b_X[1][0] * dist(e2);
      tempb = b_X[0][1] + b_X[1][1] * dist(e2);
      tempc = b_X[0][2] + b_X[1][2] * dist(e2);
      D = ((cur_M[0] + tempa*cur_M[1] + tempb*cur_M[2] + tempc) /
	   sqrt(1 + SQ(tempa) + SQ(tempb))) - R;
      if (D >= -Xstd_ob && D <= Xstd_ob)
	{
	  X[i][0] = tempa;
	  X[i][1] = tempb;
	  X[i][2] = tempc;

	  i += 1;
	}
      count += 1;
    }
  return iffar;

};

void particleFilter::calc_weight(double *W, int n_particles, double Xstd_tran,
				 cspace *X0, cspace *X)
{
  double A = 1.0 / (sqrt(2 * Pi) * Xstd_tran);
  double B = -0.5 / SQ(Xstd_tran);
  double sum = 0;
  for (int k = 0; k < n_particles; k++) {
    for (int m = 0; m < n_particles; m++) {
      W[k] += A*exp(B*(SQ(X0[m][0] - X[k][0]) + SQ(X0[m][1] - X[k][1]) +
		       SQ(X0[m][2] - X[k][2])));
    }
    sum += W[k];
  }
  for (int k = 0; k < n_particles; k++) {
    W[k] /= sum;
  }
};

void particleFilter::resample_particles(cspace *X0, cspace *X, double *W,
					int n_particles)
{
  double *Cum_sum = new double[n_particles];
  Cum_sum[0] = W[0];
  std::default_random_engine generator;
  std::uniform_real_distribution<double> rd(0,1);
  for (int i = 1; i < n_particles; i++)
    {
      Cum_sum[i] = Cum_sum[i - 1] + W[i];
		
    }
  double t;
  for (int i = 0; i < n_particles; i++)
    {
      t=rd(generator);
      for (int j = 0; j < n_particles; j++)
	{
	  if (j==0 && t <= Cum_sum[0])
	    {
	      X0[i][0] = X[0][0];
	      X0[i][1] = X[0][1];
	      X0[i][2] = X[0][2];
	    }
	  else if (Cum_sum[j-1]<t && t<= Cum_sum[j])
	    {
	      X0[i][0] = X[j][0];
	      X0[i][1] = X[j][1];
	      X0[i][2] = X[j][2];
	    }
	}
    }
}

#if 0
int main()
{
  int N = 800;
  double Xstd_ob = 0.0001;
  double Xstd_tran = 0.0025;
  double Xstd_scatter = 0.0001;

  double R = 0.01;

  particleFilter::cspace X_true = { 2.04, 1.45, 3.94 };
  cout << X_true[0] << ' ' << X_true[1] << ' ' << X_true[2] << endl;
  particleFilter::cspace b_Xprior[2] = { { 2, 1.4, 4 }, {0.1, 0.1, 0.1} };

  particleFilter pfilter(N, b_Xprior, Xstd_ob, Xstd_tran, Xstd_scatter, R);

  int N_Measure = 20;
  double M_std = 0.000000001;
  double M[3];
  particleFilter::cspace X_est[2];
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-4, 4);
  normal_distribution<double> dist(0, M_std);
  for (int i = 0; i < N_Measure; i++) {
    M[0] = distribution(generator);
    M[1] = distribution(generator);
    M[2] = (R*sqrt(1.0 + SQ(X_true[0]) + SQ(X_true[1]))
	    - X_true[2] - M[0] - X_true[0] * M[1]) / X_true[1];
    M[0] += dist(generator);
    M[1] += dist(generator);
    M[2] += dist(generator);

    cout << "New observation: " << M[0] << " " << M[1] << " " << M[2] << endl;
    pfilter.addObservation(M);
    pfilter.estimatedDistribution(X_est);
    double diff = sqrt(SQ(X_true[0] - X_est[0][0]) +
		       SQ(X_true[0] - X_est[0][0]) +
		       SQ(X_true[0] - X_est[0][0]));
    cout << "est: " << X_est[0][0] << ' ' << X_est[0][1] << ' ' << X_est[0][2]
	 << ' ' << X_est[1][0]<< " (" << diff << ")" << endl;
  }
}
#endif
