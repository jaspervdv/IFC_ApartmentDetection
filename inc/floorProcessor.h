#define IfcSchema Ifc4
#include "helper.h"

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <ifcparse/IfcFile.h>
//#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>

#include <tuple>

#ifndef FLOORPROCESSOR_HELPER_H
#define FLOORPROCESSOR_HELPER_H

class floorProcessor {

private:

	// returns a vector filled with the top faces of the present floorslab objects
	static std::vector<TopoDS_Face> getSlabFaces(helper* data);

	// returns a vector of the areas of the faces
	static std::vector<double> getFaceAreas(std::vector<TopoDS_Face> faces);

	static std::vector<double> computeElevations(std::vector<TopoDS_Face> faces);



public:

	static std::vector<double> getStoreyElevations(helper* data);
	static std::vector<double> getFloorElevations(helper* data);
	static bool compareElevations(std::vector<double> elevations, std::vector<double> floors);

	// removes all the storey data from the file
	static void cleanStoreys(helper* data);

	// adds new storeys based on the inputted vector elevations
	static void createStoreys(helper* data, std::vector<double> floorStoreys);

	static void sortObjects(helper* data);

	// TODO make private
	static void printLevels(std::vector<double> levels);
	//static void printLevels(std::vector<std::vector<double>> levels);
};

#endif // FLOORPROCESSOR_HELPER_H