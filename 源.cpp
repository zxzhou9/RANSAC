#include <iostream>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstring>
#include <string>
using namespace std;

struct sample {
	double x;
	double y;
	double z;
}r_sample[8096];

struct plane_para {
	double A_best, B_best, C_best, D_best;
	int inPlaneNum_max = 0;
	char plane_point[8096] = { 0 };
	int iterNum = 0;
};

struct plane_para plane_ransac(int RSample_pointsNum, int iter_maxNum, double maxD) {
	struct plane_para p;
	
	int inPlaneNum_t = 0;
	double A_t, B_t, C_t, D_t;
	double x1, x2, x3, y1, y2, y3, z1, z2, z3;
	double temp, temp_D;
	int rand_i_1, rand_i_2, rand_i_3;

	while ((p.iterNum < iter_maxNum)/* && p.inPlaneNum_max <= RSample_pointsNum*/)
	{
		inPlaneNum_t = 3;//当前拟合平面中点个数

		do {
			srand((unsigned)time(NULL));
			rand_i_1 = rand() % RSample_pointsNum;
			rand_i_2 = rand() % RSample_pointsNum;
			if (rand_i_1 == rand_i_2)continue;
			rand_i_3 = rand() % RSample_pointsNum;
			if (rand_i_1 == rand_i_3 || rand_i_2 == rand_i_3)continue;

			x1 = r_sample[rand_i_1].x; x2 = r_sample[rand_i_2].x; x3 = r_sample[rand_i_3].x;
			y1 = r_sample[rand_i_1].y; y2 = r_sample[rand_i_2].y; y3 = r_sample[rand_i_3].y;

		} while (((y2 - y1) * (x3 - x2)) == ((y3 - y2) * (x2 - x1)));

		z1 = r_sample[rand_i_1].z;
		z2 = r_sample[rand_i_2].z;
		z3 = r_sample[rand_i_3].z;
		//求平面方程
		A_t = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		B_t = (x3 - x1) * (z2 - z1) - (x2 - x1) * (z3 - z1);
		C_t = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
		D_t = -(A_t * x1 + B_t * y1 + C_t * z1);

		//求在平面内的点的个数
		temp = sqrt(A_t * A_t + B_t * B_t + C_t * C_t);//点到平面距离参数

		for (int i = 0; i < RSample_pointsNum; i++)
		{
			temp_D = abs(A_t * r_sample[i].x + B_t * r_sample[i].y + C_t * r_sample[i].z + D_t) / temp;//点到平面距离
			if (temp_D < maxD)
			{
				inPlaneNum_t++;
				p.plane_point[i] = 1;
			}
			else
			{
				p.plane_point[i] = 0;
			}
		}

		//与最优（最大）个数比较，保留最优个数的平面公式
		if (inPlaneNum_t > p.inPlaneNum_max)
		{
			p.A_best = A_t;
			p.B_best = B_t;
			p.C_best = C_t;
			p.D_best = D_t;
			p.inPlaneNum_max = inPlaneNum_t;
		}
		p.iterNum++;//迭代次数+1
	}
	p.D_best = abs(p.D_best) / sqrt(p.A_best * p.A_best + p.B_best * p.B_best + p.C_best * p.C_best);
	return p;
}

//struct cylinder_para {
//	double R_best;
//	int inPlaneNum_max = 0;
//	char cylinder_point[8096] = { 0 };
//	int iterNum = 0;
//	double x_mid_best, y_mid_best, z_mid_best;
//	double s_best, m_best, n_best;
//};
//
//struct cylinder_para cylinder_ransac(int RSample_pointsNum, int iter_maxNum, double maxR) {
//	struct cylinder_para c;
//
//	int inPlaneNum_t = 0;
//	double x1, x2, x3, y1, y2, y3, z1, z2, z3;
//	double temp, temp_R;
//	int rand_i_1, rand_i_2, rand_i_3;
//
//	while ((c.iterNum < iter_maxNum) && c.inPlaneNum_max <= RSample_pointsNum)
//	{
//		inPlaneNum_t = 3;//当前拟合平面中点个数
//
//		do {
//			srand((unsigned)time(NULL));
//			rand_i_1 = rand() % RSample_pointsNum;
//			rand_i_2 = rand() % RSample_pointsNum;
//			if (rand_i_1 == rand_i_2)continue;
//			rand_i_3 = rand() % RSample_pointsNum;
//			if (rand_i_1 == rand_i_3 || rand_i_2 == rand_i_3)continue;
//
//			x1 = r_sample[rand_i_1].x; x2 = r_sample[rand_i_2].x; x3 = r_sample[rand_i_3].x;
//			y1 = r_sample[rand_i_1].y; y2 = r_sample[rand_i_2].y; y3 = r_sample[rand_i_3].y;
//
//		} while (((y2 - y1) * (x3 - x2)) == ((y3 - y2) * (x2 - x1)));
//
//		z1 = r_sample[rand_i_1].z;
//		z2 = r_sample[rand_i_2].z;
//		z3 = r_sample[rand_i_3].z;
//		//求圆柱方程
//		double x_mid = (x1 + x2) / 2;
//		double y_mid = (y1 + y2) / 2;
//		double z_mid = (z1 + z2) / 2;
//		double R = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1)) / 2;
//		double s = x3 - x1;
//		double m = y3 - y1;
//		double n = z3 - z1;
//		//轴线方程 X = x_mid + st;
//		//Y = y_mid + mt;
//		//Z = z_mid + nt;
//		//求在平面内的点的个数
//		for (int i = 0; i < RSample_pointsNum; i++)
//		{
//			double s_test = r_sample[i].x - x_mid;
//			double m_test = r_sample[i].y - y_mid;
//			double n_test = r_sample[i].z - z_mid;
//			double cos_test = abs(s * s_test + m * m_test + n * n_test) / (sqrt(s * s + m * m + n * n) * sqrt(s_test * s_test + m_test * m_test + n_test * n_test));
//			double sin_test = sqrt(1 - cos_test * cos_test);
//			double temp_R;
//			temp_R = sin_test * sqrt(s_test * s_test + m_test * m_test + n_test * n_test);
//
//			if (abs(temp_R-R) < maxR)
//			{
//				inPlaneNum_t++;
//				c.cylinder_point[i] = 1;
//			}
//			else
//			{
//				c.cylinder_point[i] = 0;
//			}
//		}
//
//		//与最优（最大）个数比较，保留最优个数的平面公式
//		if (inPlaneNum_t > c.inPlaneNum_max)
//		{
//			c.R_best = R;
//			c.x_mid_best = x_mid;
//			c.y_mid_best = y_mid;
//			c.z_mid_best = z_mid;
//			c.inPlaneNum_max = inPlaneNum_t;
//			c.s_best = s;
//			c.m_best = m;
//			c.n_best = n;
//		}
//		c.iterNum++;//迭代次数+1
//	}
//	return c;
//}

int main() {
	fstream file("D:\\2.txt");
	vector<string> lines;
	string line;
	stringstream s;
	float x, y, z;
	while (getline(file, line)) {
		lines.push_back(line);
	}
	for (unsigned int i = 0; i < lines.size(); i++) {
		s.str("");
		s.clear();
		s << lines[i];
		s >> x >> y >> z;
		r_sample[i].x = x;
		r_sample[i].y = y;
		r_sample[i].z = z;
	}
	struct plane_para p = plane_ransac(8096, 5000, 0.01);
	//struct cylinder_para c = cylinder_ransac(8096, 1000000, 0.01);

	FILE* fp1 = fopen("D:\\1_.txt", "a+");
	fprintf(fp1, "----0\n");
	fprintf(fp1, "--1\n");
	fprintf(fp1, "%f %f %f\n", p.A_best,p.B_best,p.C_best);
	fprintf(fp1, "%f\n%u\n", p.D_best,p.inPlaneNum_max);
	for (int i = 0; i < lines.size(); i++) {
		if (p.plane_point[i] == 1) {
			fprintf(fp1, "%u ", i);
		}
	}
	fclose(fp1);

	/*FILE* fp2 = fopen("D:\\2_.txt", "a+");
	fprintf(fp2, "----0\n");
	fprintf(fp2, "--2\n");
	fprintf(fp2, "%f %f %f\n", c.x_mid_best, c.cylinder_point, c.z_mid_best);
	fprintf(fp2, "%f %f %f\n%u\n", c.s_best,c.m_best,c.n_best, c.inPlaneNum_max);
	for (int i = 0; i < lines.size(); i++) {
		if (c.cylinder_point[i] == 1) {
			fprintf(fp2, "%u ", i);
		}
	}
	fclose(fp2);*/
	return 0;
}