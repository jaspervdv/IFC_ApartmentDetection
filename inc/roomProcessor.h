#include "helper.h"
#include "floorProcessor.h"

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Sewing.hxx>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <vector>
#include <tuple>

class voxel {
private:
	bool isIntersecting_ = false;
	bool isInside = true;
	std::vector<int> roomnums_;
	BoostPoint3D center_;
	double size_;

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

	void addRoomNumber(int num) { roomnums_.emplace_back(num); }

	void setOutside() { isInside = false; }

	std::vector<int> getRoomNumbers() { return roomnums_; }

	bool getIsInside() { return isInside; }

	BoostPoint3D getCenterPoint() { return center_; }

	BoostPoint3D getCenterPoint(double angle) { return rotatePointWorld(center_, -angle); }

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
	double voxelSize_ = 0.44;

	double planeRotation_ = 0;

	// -1 is intersected 0 is not assigned 1..n is room assignement;
	std::vector<int> Assignment_;
	std::map<int, voxel*> VoxelLookup_;
	
	std::vector<int> getNeighbours(int voxelIndx);

	// transform coordinates 
	template<typename T>
	T linearToRelative(int i);
	BoostPoint3D relPointToWorld(BoostPoint3D p);
	BoostPoint3D relPointToWorld(int px, int py, int pz);
	BoostPoint3D WorldPointToRel(BoostPoint3D p);

	// create a group of voxels representing a rough room
	std::vector<int> growRoom(int startIndx, int roomnum);

	// generates the faces of the voxel that are needed to create a rough room shape
	std::vector<TopoDS_Face> getPartialFaces(std::vector<int> roomIndx, int voxelIndx);

	// creates and adds a voxel object + checks with which products from the cluster it intersects
	void addVoxel(int indx, helperCluster* cluster);

	void outputFieldToFile();

public:

	explicit voxelfield(helperCluster* cluster);

	void makeRooms(helperCluster* cluster);
};