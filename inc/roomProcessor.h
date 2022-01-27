#define IfcSchema Ifc4

#include "helper.h"

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <vector>


class voxel {
private:
	bool isIntersecting = false;
	
public:

	void checkIntersecting();

	bool getIsIntersecting() { return isIntersecting; }
};

class voxelfield {
	
private:
	//world space data
	gp_Pnt anchor_;
	double xRange_;
	double yRange_;
	double zRange_;

	// relative space data
	double xRelRange_;
	double yRelRange_;
	double zRelRange_;

	std::vector<double> voxelSize_ = { 0.0, 0.0, 0.0 };

	double desiredVSize_ = 1;

public:

	explicit voxelfield(helperCluster* cluster);

	static void makeRooms(helperCluster* cluster, voxelfield field);
};