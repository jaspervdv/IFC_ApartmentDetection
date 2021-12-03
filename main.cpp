// TODO: Multiple schemas
#define IfcSchema Ifc2x3

#include "inc/helper.h"
#include "inc/floorProcessor.h"

// basic includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>

// boost includes
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

// openCASCADE includes
#include <TopoDS.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

// IfcOpenShell includes
#include <ifcparse/IfcFile.h>
#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>

#if USE_VLD
#include <vld.h>
#endif

int main(int argc, char** argv) {
	Logger::SetOutput(&std::cout, &std::cout);

	std::string sourcePath = "D:/Documents/Uni/Thesis/sources/Models/Rotterdam/9252_VRI_Boompjes_constructie.ifc";
	std::string exportPath = "D:/Documents/Uni/Thesis/sources/Models/exports/AC20-FZK-Haus.ifc";

	// initialize helper
	helper* hFile1 = new helper(sourcePath);

	// check if units have been set up correctly
	if (!hFile1->hasSetUnits()) { return 0; }

	// create working basefile
	IfcHierarchyHelper<IfcSchema> workingFile;
	workingFile.header().file_name().name("test.ifc");

	IfcSchema::IfcBuildingElement::list::ptr elements = hFile1->getSourceFile()->instances_by_type<IfcSchema::IfcBuildingElement>();
	std::cout << "Found " << elements->size() << " elements " << std::endl;

	//TODO clash detection

	IfcSchema::IfcBuildingStorey::list::ptr storeys = hFile1->getSourceFile()->instances_by_type<IfcSchema::IfcBuildingStorey>();
	std::vector<double> floorArea;
	std::vector<double> storeyElevation;
	std::vector<double> floorElevation;
	std::vector<TopoDS_Face> floorFaces;

	std::vector<double> storeyElevation2 = floorProcessor::getStoreyElevations(hFile1);
	std::vector<std::vector<double>> floorElevation2 = floorProcessor::getFloorElevations(hFile1);

	floorProcessor::compareElevations(storeyElevation2, floorElevation2);
	
	// write to file
	/*
	std::ofstream storageFile;
	storageFile.open(exportPath);
	std::cout << "exporting" << std::endl;
	storageFile << sourceFile;
	std::cout << "exported succesfully" << std::endl;
	storageFile.close();

	std::cout << "last line executed" << std::endl;
	*/
	return 0;

}