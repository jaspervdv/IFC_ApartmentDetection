#define IfcSchema Ifc4

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <ifcparse/IfcFile.h>
#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>

#include <memory>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 3, bg::cs::cartesian> BoostPoint3D;
typedef std::pair<bg::model::box<BoostPoint3D>, int> Value;
typedef std::tuple<IfcSchema::IfcProduct*, std::vector<std::vector<gp_Pnt>>> LookupValue;

#ifndef HELPER_HELPER_H
#define HELPER_HELPER_H

// Forward Decleration helper class
class helper;

// helper functions that can be utilised everywhere
gp_Pnt rotatePointWorld(gp_Pnt p, double angle);

void printPoint(gp_Pnt p);

BoostPoint3D Point3DOTB(gp_Pnt oP);

gp_Pnt Point3DBTO(BoostPoint3D oP);

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

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	// The needed rotation for the model to be aligned to the world axis!
	double originRot_;

	std::string path_;
	std::string fileName_;

	IfcParse::IfcFile* file_;
	IfcGeom::Kernel* kernel_;

	bgi::rtree<Value, bgi::rstar<25>> index_; 
	std::vector<LookupValue> productLookup_;

	// finds the ifc schema that is used in the supplied file
	void findSchema(std::string path);

	// sets the unit multipliers to allow for the use of other units than metres
	void setUnits(IfcParse::IfcFile* file);

	// returns a list of all the points present in a model
	std::vector<gp_Pnt> getAllPoints(IfcSchema::IfcProduct::list::ptr products);

	// returns a bbox of a ifcproduct that functions with boost
	bg::model::box <BoostPoint3D> makeObjectBox(const IfcSchema::IfcProduct* product);

	std::vector<std::vector<gp_Pnt>> triangulateProduct(IfcSchema::IfcProduct* product);

	template <typename T>
	void addObjectToIndex(T object);

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

	gp_Pnt getLllPoint() { return lllPoint_; }

	gp_Pnt getUrrPoint() { return urrPoint_; }

	double getRotation() { return originRot_; }

	const bgi::rtree<Value, bgi::rstar<25>>* getIndexPointer() { return &index_; }

	auto getLookup(int i) { return productLookup_[i]; }

	std::vector<gp_Pnt> getObjectPoints(const IfcSchema::IfcProduct* product, bool sortEdges = false);

	std::vector<TopoDS_Face> getObjectFaces(const IfcSchema::IfcProduct* product);

	TopoDS_Shape getObjectShape(const IfcSchema::IfcProduct* product);

	void setIsConstruct(bool b) { isConstruct = b; }
	
	void setPath(std::string path) { path_ = path; }

	void setName(std::string name) { fileName_ = name; }

	//TODO implement
	
	// add bounding box items for the present objects in the data
	void createBounds(helper* data);

	// deletes all dependencies of an object and the object itself
	static void wipeObject(helper* data, int id);

	void writeToFile(std::string path);

	void setDepending(bool i) { isPartial = i; }

	~helper() {};

};


#endif // HELPER_HELPER_H