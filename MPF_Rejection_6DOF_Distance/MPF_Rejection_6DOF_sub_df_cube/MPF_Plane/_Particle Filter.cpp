#include<iostream>
#include <random>
#include<cmath>
using namespace std;	

# define Pi          3.141592653589793238462643383279502884L

void create_particles(double (*X)[3], double b_Xprior[2][3], int n_particles);
bool update_particles(double (*X)[3], double b_Xprior[2][3], double b_Xpre[2][3], double Xstd_ob, double R, double cur_M[3], int n_particles);
void calc_weight(double *W, int n_particles, double Xstd_tran, double(*X0)[3], double(*X)[3]);
void resample_particles(double(*X0)[3], double(*X)[3], double *W, int n_particles);
int main()
{
	int N = 800;
	double Xstd_ob = 0.0001;
	double Xstd_tran = 0.0025;
	double Xstd_scatter = 0.0001;

	double R = 0.01;

	double X_true[3] = { 2.1, 1.45, 3.9 };
	cout << X_true[0] << ' ' << X_true[1] << ' ' << X_true[2] << endl;
	double b_Xprior[2][3] = { { 2, 1.4, 4 }, {0.1, 0.1, 0.1} };
	double(*b_Xpre)[3] = new double[2][3];
	memcpy(b_Xpre, b_Xprior, sizeof(b_Xprior));


	double(*X)[3] = new double[N][3];
	double(*X0)[3] = new double[N][3];
	for (int i = 0; i < N; i++)
	{
		X[i][0] = 0;
		X[i][1] = 0;
		X[i][2] = 0;
	}
	//create_particles(X, b_Xprior, N);
	double(*Xpre_weight)[3] = new double[N][3];
	int N_Measure = 100;
	double M_std = 0.000000001;
	double(*M)[3] = new double[N_Measure][3];
	double *W = new double[N];
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-4, 4);
	normal_distribution<double> dist(0, M_std);
	for (int i = 0; i < N_Measure; i++)
	{
		M[i][0] = distribution(generator);
		M[i][1] = distribution(generator);
		M[i][2] = (R*sqrt(1.0 + X_true[0] * X_true[0] + X_true[1] * X_true[1]) - X_true[2] - M[i][0] - X_true[0] * M[i][1]) / X_true[1] + dist(generator);
		M[i][0] = M[i][0] + dist(generator);
		M[i][1] = M[i][1] + dist(generator);
	}
	double(*X_est)[5] = new double[N_Measure][5];
	normal_distribution<double> dist2(0, Xstd_scatter);
	for (int i = 0; i < N_Measure; i++)
	{
		if (i > 0)
		{
			b_Xpre[0][0] = 0;
			b_Xpre[0][1] = 0;
			b_Xpre[0][2] = 0;
			b_Xpre[1][0] = 0;
			b_Xpre[1][1] = 0;
			b_Xpre[1][2] = 0;
			for (int j = 0; j < N_Measure; j++)
			{
				X0[j][0] = X0[j][0] + dist2(generator);
				X0[j][1] = X0[j][1] + dist2(generator);
				X0[j][2] = X0[j][2] + dist2(generator);
				b_Xpre[0][0] += X0[j][0] / N_Measure;
				b_Xpre[0][1] += X0[j][1] / N_Measure;
				b_Xpre[0][2] += X0[j][2] / N_Measure;
			}
			for (int j = 0; j < N_Measure; j++)
			{
				b_Xpre[1][0] += (X0[j][0] - b_Xpre[0][0])* (X0[j][0] - b_Xpre[0][0]) / N;
				b_Xpre[1][1] += (X0[j][1] - b_Xpre[0][1])* (X0[j][1] - b_Xpre[0][1]) / N;
				b_Xpre[1][2] += (X0[j][2] - b_Xpre[0][2])* (X0[j][2] - b_Xpre[0][2]) / N;
			}
			b_Xpre[1][0] = sqrt(b_Xpre[1][0]);
			b_Xpre[1][1] = sqrt(b_Xpre[1][1]);
			b_Xpre[1][2] = sqrt(b_Xpre[1][2]);
		}
		bool iffar = update_particles(X, b_Xprior, b_Xpre, Xstd_ob, R, M[i], N);
		if (i == 0)
		{
			for (int j = 0; j < N; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					X0[j][k] = X[j][k];
					Xpre_weight[j][k] = X[j][k];
				}
			}
			/*memcpy(X0, X, sizeof(X));
			memcpy(Xpre_weight, X0, sizeof(X0));*/
		}
		else if (iffar == true)
		{
			/*memcpy(X0, Xpre_weight, sizeof(Xpre_weight));*/
			for (int j = 0; j < N; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					X0[j][k] = Xpre_weight[j][k];
				}
			}
		}
		calc_weight(W, N, Xstd_tran, X0, X);
		//memcpy(Xpre_weight, X0, sizeof(X0));
		for (int j = 0; j < N; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				Xpre_weight[j][k] = X0[j][k];
			}
		}
		resample_particles(X0, X, W, N);
		X_est[i][0] = 0;
		X_est[i][1] = 0;
		X_est[i][2] = 0;
		X_est[i][3] = 0;
		X_est[i][4] = 0;
		for (int j = 0; j < N; j++)
		{
			X_est[i][0] += X0[j][0] / N;
			X_est[i][1] += X0[j][1] / N;
			X_est[i][2] += X0[j][2] / N;
		}
		for (int j = 0; j < N; j++)
		{
			X_est[i][4] += (pow(X0[j][0] - X_est[i][0],2) + pow(X0[j][1] - X_est[i][1],2) + pow(X0[j][2] - X_est[i][2],2)) / N;
		}
		X_est[i][4] = sqrt(X_est[i][4]);
		cout << X_est[i][0] << ' ' << X_est[i][1] << ' ' << X_est[i][2] << endl;
		if (X_est[i][4] < 0.005 && (abs(X_est[i][0]-b_Xpre[0][0])>0.001 || abs(X_est[i][1] - b_Xpre[0][1])>0.001 || abs(X_est[i][2] - b_Xpre[0][2])>0.001 ))
			Xstd_scatter = 0.01;
		else
			Xstd_scatter = 0.0001;
	}
};

void create_particles(double (*X)[3], double b_Xprior[2][3], int n_particles)
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

bool update_particles(double(*X)[3], double b_Xprior[2][3], double b_Xpre[2][3], double Xstd_ob, double R, double cur_M[3], int n_particles)
{
	random_device rd;

	mt19937 e2(rd());
	normal_distribution<double> dist(0, 1);
	int i = 0;
	int count = 0;
	int factor = 0;
	int nIter = n_particles*3000;
	bool iffar = false;
	double (*b_X)[3] = b_Xpre;
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
		D = ((cur_M[0] + tempa*cur_M[1] + tempb*cur_M[2] + tempc) / sqrt(1 + tempa*tempa + tempb*tempb)) - R;
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

void calc_weight(double *W, int n_particles, double Xstd_tran, double(*X0)[3], double(*X)[3])
{
	double A = 1.0 / (sqrt(2 * Pi) * Xstd_tran);
	double B = -0.5 / (Xstd_tran* Xstd_tran);
	double sum = 0;
	for (int k = 0; k < n_particles; k++)
	{
		for (int m = 0; m < n_particles; m++)
		{
			W[k] += A*exp(B*(pow(X0[m][0] - X[k][0], 2) + pow(X0[m][1] - X[k][1], 2) + pow(X0[m][2] - X[k][2], 2)));
		}
		sum += W[k];
	}
	for (int k = 0; k < n_particles; k++)
	{
		W[k] /= sum;
	}
};

void resample_particles(double(*X0)[3], double(*X)[3], double *W, int n_particles)
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