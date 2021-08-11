#ifndef _JPS_UTILS_H_
#define _JPS_UTILS_H_

#include <iostream>
///Search and prune neighbors for JPS 3D
struct JPS3DNeib {
	// for each (dx,dy,dz) these contain:
	//    ns: neighbors that are always added
	//    f1: forced neighbors to check
	//    f2: neighbors to add if f1 is forced
	int ns[27][3][26];//用来存储一个3x3x3的栅格中，每一个点的邻居
	int f1[27][3][12];//用来存储将要进行Forced neighbor判断的点
	int f2[27][3][12];//用来存储经过判断确实是Forced neighbor的点
	// nsz contains the number of neighbors for the four different types of moves:
	// no move (norm 0):        26 neighbors always added
	//                          0 forced neighbors to check (never happens)
	//                          0 neighbors to add if forced (never happens)
	// straight (norm 1):       1 neighbor always added
	//                          8 forced neighbors to check
	//                          8 neighbors to add if forced
	// diagonal (norm sqrt(2)): 3 neighbors always added
	//                          8 forced neighbors to check
	//                          12 neighbors to add if forced
	// diagonal (norm sqrt(3)): 7 neighbors always added
	//                          6 forced neighbors to check
	//                          12 neighbors to add if forced
	// 上面所述的原理，实际上类比于二维情况还是比较容易理解的
	static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
	JPS3DNeib();
	private:
	void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
	void FNeib( int dx, int dy, int dz, int norm1, int dev,
	    int& fx, int& fy, int& fz,
	    int& nx, int& ny, int& nz);
};

#endif