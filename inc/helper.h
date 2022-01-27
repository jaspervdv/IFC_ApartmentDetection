#define IfcSchema Ifc4

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <ifcparse/IfcFile.h>
#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>

#include <memory>

#ifndef HELPER_HELPER_H
#define HELPER_HELPER_H

// Forward Decleration helper class
class helper;

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
	bool isConstruct = false; //TODO implement
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

	// finds the ifc schema that is used in the supplied file
	void findSchema(std::string path);

	// sets the unit multipliers to allow for the use of other units than metres
	void setUnits(IfcParse::IfcFile* file);

	std::vector<gp_Pnt> getAllPoints(IfcSchema::IfcProduct::list::ptr products);

public:
	
	/* 
	construct and populate a helper
	creates and stores SI unit mulitpliers for length, area and volume
	creates and stores the file and kernel for quick acess
	*/
	explicit helper(std::string path);

	// returns true when length, area and volume multiplier are not 0
	bool hasSetUnits();

	void internalizeGeo();

	void internalizeGeo(double angle);

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