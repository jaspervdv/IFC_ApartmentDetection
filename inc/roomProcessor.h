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
	bg::model::point<float, 3, bg::cs::cartesian> center_;
	float size_;

public:

	explicit voxel(bg::model::point<float, 3, bg::cs::cartesian> center, double size);

	bg::model::box<bg::model::point<float, 3, bg::cs::cartesian>> getVoxelGeo();

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
	int xRelRange_;
	int yRelRange_;
	int zRelRange_;

	int totalVoxels_;

	// x y z size of the voxel
	double voxelSize_ = 1;

	bg::model::point<float, 3, bg::cs::cartesian> relPointToWorld(bg::model::point<float, 3, bg::cs::cartesian> p);
	bg::model::point<float, 3, bg::cs::cartesian> relPointToWorld(int px, int py, int pz);

	bg::model::point<float, 3, bg::cs::cartesian> WorldPointToRel(bg::model::point<float, 3, bg::cs::cartesian> p);

public:

	explicit voxelfield(helperCluster* cluster);

	void makeRooms(helperCluster* cluster);
};