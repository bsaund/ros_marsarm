#include<iostream>
using namespace std;
void process_2d_array(int(*array)[10])
{
	int rows = 5;
	std::cout << __func__ << std::endl;
	for (size_t i = 0; i < rows; ++i)
	{
		std::cout << i << ": ";
		for (size_t j = 0; j < 10; ++j)
			std::cout << array[i][j] << '\t';
		std::cout << std::endl;
	}
}
int main()
{
	int a[5][10] = { {} };
	process_2d_array(a);
}