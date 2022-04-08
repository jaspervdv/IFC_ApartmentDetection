#define USE_IFC4

#ifdef USE_IFC4
#define IfcSchema Ifc4
#else
#define IfcSchema Ifc2x3
#endif // USE_IFC4

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <ifcparse/IfcFile.h>
#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>
#include <ifcgeom_schema_agnostic/Serialization.h>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>

#include <memory>

// Forward Decleration helper class
class helper;
struct roomObject;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 3, bg::cs::cartesian> BoostPoint3D;
typedef std::pair<bg::model::box<BoostPoint3D>, int> Value;
typedef std::tuple<IfcSchema::IfcProduct*, std::vector<std::vector<gp_Pnt>>, bool, TopoDS_Shape*> LookupValue;
typedef std::tuple<IfcSchema::IfcProduct*,std::vector<gp_Pnt>, std::vector<roomObject*>*> ConnectLookupValue;
typedef std::tuple<IfcSchema::IfcSpace*, TopoDS_Shape> roomLookupValue;

#ifndef HELPER_HELPER_H
#define HELPER_HELPER_H

// helper functions that can be utilised everywhere
gp_Pnt rotatePointWorld(gp_Pnt p, double angle);
BoostPoint3D rotatePointWorld(BoostPoint3D p, double angle);

void WriteToSTEP(TopoDS_Solid shape, std::string addition);
void WriteToSTEP(TopoDS_Shape shape, std::string addition);

void printPoint(gp_Pnt p);
void printPoint(BoostPoint3D p);

void printFaces(TopoDS_Shape shape);

BoostPoint3D Point3DOTB(gp_Pnt oP);

gp_Pnt Point3DBTO(BoostPoint3D oP);

gp_Pnt getLowestPoint(TopoDS_Shape shape, bool areaFilter);
gp_Pnt getHighestPoint(TopoDS_Shape shape);

std::vector<IfcSchema::IfcProduct*> getNestedProducts(IfcSchema::IfcProduct* product);

class helperCluster
{
private:
	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;
	double originRot_;

	bool hasBbox_ = false;

	std::vector<helper*> helperList;
	int size_ = 0;

public:
	std::vector<helper*> getHelper() const { return helperList; }

	gp_Pnt getLllPoint() const { return lllPoint_; }
	gp_Pnt getUrrPoint() const { return urrPoint_; }
	double getDirection() const { return originRot_; }

	int getSize() const { return size_; }

	void internaliseData();

	bool hasBbox() const { return hasBbox_; }

	void appendHelper(std::string path);
	void appendHelper(helper* data);

	void makeBbox();

	helper* getHelper(int i) { return helperList[i]; }
	std::vector<helper*> getHelpers() { return helperList; }

};

class helper
{
private:

	// The unit multipliers found
	double length_ = 0;
	double area_ = 0;
	double volume_ = 0;

	bool hasFloors = false;
	bool isConstruct = false; 
	bool isPartial = false;
	bool hasGeo = false;
	bool hasRooms = false;

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	// The needed rotation for the model to be aligned to the world axis!
	double originRot_;

	std::string path_;
	std::string fileName_;

	IfcParse::IfcFile* file_;
	IfcGeom::Kernel* kernel_;

	static const int treeDepth = 25;
	bgi::rtree<Value, bgi::rstar<treeDepth>> index_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> cIndex_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> rIndex_;
	std::vector<LookupValue> productLookup_;
	std::vector<ConnectLookupValue> connectivityLookup_;
	std::vector<roomLookupValue> roomLookup_;
	std::vector<gp_Pnt> roomCenterPoints_;

	// finds the ifc schema that is used in the supplied file
	void findSchema(std::string path);

	// sets the unit multipliers to allow for the use of other units than metres
	void setUnits(IfcParse::IfcFile* file);

	// returns a list of all the points present in a model
	std::vector<gp_Pnt> getAllPoints(IfcSchema::IfcProduct::list::ptr products);

	// returns a bbox of a ifcproduct that functions with boost
	bg::model::box <BoostPoint3D> makeObjectBox(IfcSchema::IfcProduct* product);
	bg::model::box <BoostPoint3D> makeObjectBox(std::vector<IfcSchema::IfcProduct*> products);

	std::vector<std::vector<gp_Pnt>> triangulateProduct(IfcSchema::IfcProduct* product);

	template <typename T>
	void addObjectToIndex(T object);

	template <typename T>
	void addObjectToCIndex(T object);

	template <typename T>
	void addObjectToRIndex(T object);

public:
	
	/* 
	construct and populate a helper
	creates and stores SI unit mulitpliers for length, area and volume
	creates and stores the file and kernel for quick acess
	*/
	explicit helper(std::string path);

	// returns true when length, area and volume multiplier are not 0
	bool hasSetUnits();

	// internalises the geometry while approximating a smallest bbox around the geometry
	void internalizeGeo();

	// internalises the geometry while creating a bbox with one axis along the give angle
	void internalizeGeo(double angle);

	// makes a spatial index for the geometry
	void indexGeo();

	// corrects room classification
	void correctRooms();

	// returns a vector with length, area and volume multipliers
	std::vector<double> getUnits() const { return { length_, area_, volume_ }; }

	// returns the length multiplier
	double getLengthMultiplier() const { return length_; }

	// returns the area multiplier
	double getAreaMultiplier() const { return area_; }

	// returns the volume multiplier
	double getVolumeMultiplier() const { return volume_; }

	std::string getName() const { return fileName_; }

	std::string getPath() const { return path_; }

	// returns a pointer to the sourcefile
	IfcParse::IfcFile* getSourceFile() const { return file_; }

	// returns a pointer to the kernel
	IfcGeom::Kernel* getKernel() const { return kernel_; }

	// returns a pointer to the owner(s)
	IfcSchema::IfcOwnerHistory* getHistory();

	bool getDepending() { return isPartial; }

	bool getIsConstruct() { return isConstruct; }

	bool getHasGeo() { return hasGeo; }

	bool getHasRoom() { return hasRooms; }

	gp_Pnt getLllPoint() { return lllPoint_; }

	gp_Pnt getUrrPoint() { return urrPoint_; }

	double getRotation() { return originRot_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getIndexPointer() { return &index_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getConnectivityIndexPointer() { return &cIndex_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getRoomIndexPointer() { return &rIndex_; }

	auto getLookup(int i) { return productLookup_[i]; }
	
	auto getCLookup(int i) { return connectivityLookup_[i]; }

	auto getRLookup(int i) { return roomLookup_[i]; }

	auto getFullClookup() { return connectivityLookup_; }

	auto getFullRLookup() { return roomLookup_; }

	auto getRoomCenters() { return roomCenterPoints_; }

	std::vector<gp_Pnt> getObjectPoints(IfcSchema::IfcProduct* product, bool sortEdges = false);

	std::vector<TopoDS_Face> getObjectFaces(IfcSchema::IfcProduct* product);

	TopoDS_Shape getObjectShape(IfcSchema::IfcProduct* product);

	void setIsConstruct(bool b) { isConstruct = b; }
	
	void setPath(std::string path) { path_ = path; }

	void setName(std::string name) { fileName_ = name; }

	void setHasRooms() { hasRooms = true; }

	//TODO implement
	void whipeObject(IfcSchema::IfcProduct* product);
	
	// add bounding box items for the present objects in the data
	void createBounds(helper* data);

	// deletes all dependencies of an object and the object itself
	static void wipeObject(helper* data, int id);

	void writeToFile(std::string path);

	void setDepending(bool i) { isPartial = i; }

	~helper() {};

};

class roomObject {
private:
	IfcSchema::IfcSpace* self_;
	std::vector<roomObject*> connections_;
	gp_Pnt point_;
	int indexNum_;
	int sectionNum_ = -1;
	bool isInside_ = true;
public:

	roomObject(IfcSchema::IfcSpace* s, int i) { self_ = s; point_ = gp_Pnt(i, 0, 0); indexNum_ = i; }

	void setSelf(IfcSchema::IfcSpace* s) { self_ = s; }

	void setPoint(int i) { point_ = gp_Pnt(i, 0, 0); }

	void setIndx(int i) { indexNum_ = i; }

	void setIsOutSide() { isInside_ = false; }

	void setSNum(int i) { sectionNum_ = i; }

	IfcSchema::IfcSpace* getSelf() { return self_; }

	const std::vector<roomObject*> getConnections() { return connections_; }

	void addConnection(roomObject* product) { connections_.emplace_back(product); }

	gp_Pnt getPoint() { return point_; }

	int getIdx() { return indexNum_; }

	int getSNum() { return sectionNum_; }

	bool isInside() { return isInside_; }

};


#endif // HELPER_HELPER_H
