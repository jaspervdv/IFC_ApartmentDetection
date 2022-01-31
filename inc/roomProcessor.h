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
	BoostPoint3D center_;
	float size_;

public:

	explicit voxel(BoostPoint3D center, double size);

	bg::model::box<BoostPoint3D> getVoxelGeo();

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


	// -1 is intersected 0 is not assigned 1..n is room assignement;
	std::vector<int> Assignment;

	template<typename T>
	T linearToRelative(int i);

	BoostPoint3D relPointToWorld(BoostPoint3D p);
	BoostPoint3D relPointToWorld(int px, int py, int pz);

	BoostPoint3D WorldPointToRel(BoostPoint3D p);

public:

	explicit voxelfield(helperCluster* cluster);

	void makeRooms(helperCluster* cluster);
};