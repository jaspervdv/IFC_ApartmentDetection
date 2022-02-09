#define IfcSchema Ifc4

#include "helper.h"

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <vector>
#include <tuple>

class voxel {
private:
	bool isIntersecting_ = false;
	std::vector<int> roomnums = { 0 };
	BoostPoint3D center_;
	double size_;

	std::vector<IfcSchema::IfcProduct*> intersectingProducts;

	// compute the signed volume
	double tVolume(gp_Pnt p, const std::vector<gp_Pnt> vertices);

	bool checkIntersecting(const std::vector<gp_Pnt> line, const std::vector<gp_Pnt> triangle);

public:

	// greates an axis aligned voxel
	explicit voxel(BoostPoint3D center, double size);

	// returns the lll and urr point of a voxel in axis aligned space
	bg::model::box<BoostPoint3D> getVoxelGeo();

	// return the cornerpoints of a voxel based on the angle
	std::vector<gp_Pnt> getCornerPoints(double angle);

	// returns integers with that comply with the getCornerPoints output
	std::vector<std::vector<int>> getVoxelTriangles();
	std::vector<std::vector<int>> getVoxelFaces();
	std::vector<std::vector<int>> getVoxelEdges();

	// returns the products that intersect with the voxel
	std::vector<IfcSchema::IfcProduct*> getProducts() { return intersectingProducts; }

	// add product to the voxel
	void addProduct(IfcSchema::IfcProduct* product) { intersectingProducts.emplace_back(product); }

	// check the intersection of a triangluted product and a voxel
	bool checkIntersecting(LookupValue lookup, const std::vector<gp_Pnt> voxelPoints, helper* h);

	bool linearEqIntersection(std::vector<gp_Pnt> productPoints, std::vector<gp_Pnt> voxelPoints);

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
	double voxelSize_ = 1;

	double planeRotation_ = 0;

	// -1 is intersected 0 is not assigned 1..n is room assignement;
	std::vector<int> Assignment;
	std::map<int, voxel> VoxelLookup;

	std::vector<int> getNeighbours(int voxelIndx);

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