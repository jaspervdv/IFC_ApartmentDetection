#define IfcSchema Ifc2x3

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

#ifndef FLOORPROCESSOR_HELPER_H
#define FLOORPROCESSOR_HELPER_H

class floorProcessor {

private:

	// returns a vector filled with the top faces of the present floorslab objects
	static std::vector<TopoDS_Face> getSlabFaces(helper* data);

	// returns a vector of the areas of the faces
	static std::vector<double> getFaceAreas(std::vector<TopoDS_Face> faces);
	static std::vector<int> getPairing(std::vector<TopoDS_Face> faces);
	static std::vector<std::vector<double>> getLevel(std::vector<TopoDS_Face> faces, std::vector<int> pairs);

public:

	static std::vector<double> getStoreyElevations(helper* data);
	static std::vector<std::vector<double>> getFloorElevations(helper* data);
	static void compareElevations(std::vector<double> elevations, std::vector<std::vector<double>> floors);


};

#endif // FLOORPROCESSOR_HELPER_H