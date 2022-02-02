#define IfcSchema Ifc2x3

#include "helper.h"

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <vector>


class voxel {
private:
	bool isIntersecting_ = false;
	BoostPoint3D center_;
	double size_;

	// triangulated boxel indecies
	std::vector<std::vector<int>> triangulationBoxel = {
		{ 0, 1, 5 }, // side	
		{ 0, 5, 6 },
		{ 1, 2, 4 },
		{ 1, 4, 5 },
		{ 2, 3, 7 },
		{ 2, 7, 4 },
		{ 3, 0, 6 },
		{ 3, 6, 7 },
		{ 6, 5, 4 }, // top
		{ 6, 4, 7 },
		{ 0, 3, 2 }, // buttom
		{ 0, 2, 1 }
	};
public:

	explicit voxel(BoostPoint3D center, double size);

	bg::model::box<BoostPoint3D> getVoxelGeo();
	std::vector<gp_Pnt> getCornerPoints(double angle);

	double tVolume(gp_Pnt p, const std::vector<gp_Pnt> vertices);

	void checkIntersecting(const IfcSchema::IfcProduct* product, helper* h, std::vector<gp_Pnt> voxelPoints);

	bool getIsIntersecting() { return isIntersecting_; }
};

class voxelfield {

private:
	//world space data
	gp_Pnt anchor_;

	// relative space data
	int xRelRange_;
	int yRelRange_;
	int zRelRange_;

	int totalVoxels_;

	// x y z size of the voxel
	double voxelSize_ = 2;

	double planeRotation_ = 0;

	// -1 is intersected 0 is not assigned 1..n is room assignement;
	std::vector<int> Assignment;
	std::map<int, voxel> VoxelLookup;

	template<typename T>
	T linearToRelative(int i);

	BoostPoint3D relPointToWorld(BoostPoint3D p);
	BoostPoint3D relPointToWorld(int px, int py, int pz);

	BoostPoint3D WorldPointToRel(BoostPoint3D p);

	void outputFieldToFile();

public:

	explicit voxelfield(helperCluster* cluster);

	void makeRooms(helperCluster* cluster);
};