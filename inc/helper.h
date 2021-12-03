#define IfcSchema Ifc2x3

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <ifcparse/IfcFile.h>
//#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>

#ifndef HELPER_HELPER_H
#define HELPER_HELPER_H

class helper
{
private:

	// The unit multipliers found
	double length_ = 0;
	double area_ = 0;
	double volume_ = 0;

	IfcParse::IfcFile* file_;
	IfcGeom::Kernel* kernel_;

	// finds the ifc schema that is used in the supplied file
	void findSchema(std::string path);

	// sets the unit multipliers to allow for the use of other units than metres
	void setUnits(IfcParse::IfcFile* file);

public:
	
	/* 
	construct and populate a helper
	creates and stores SI unit mulitpliers for length, area and volume
	creates and stores the file and kernel for quick acess
	*/
	explicit helper(std::string path);

	// returns true when length, area and volume multiplier are not 0
	bool hasSetUnits();

	// returns a vector with length, area and volume multipliers
	std::vector<double> getUnits() const { return { length_, area_, volume_ }; }

	// returns the length multiplier
	double getLengthMultiplier() const { return length_; }

	// returns the area multiplier
	double getAreaMultiplier() const { return area_; }

	// returns the volume multiplier
	double getVolumeMultiplier() const { return volume_; }

	// returns a pointer to the sourcefile
	IfcParse::IfcFile* getSourceFile() const { return file_; }

	// returns a pointer to the kernel
	IfcGeom::Kernel* getKernel() const { return kernel_; }
};

#endif // HELPER_HELPER_H