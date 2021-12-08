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

typedef IfcParse::IfcGlobalId guid;

int main(int argc, char** argv) {
	Logger::SetOutput(&std::cout, &std::cout);

	std::string sourcePath = "D:/Documents/Uni/Thesis/sources/Models/Rotterdam/9252_VRI_Boompjes_constructie.ifc";
	std::string exportPath = "D:/Documents/Uni/Thesis/sources/Models/exports/AC20-FZK-Haus.ifc";

	std::vector<std::string> sourcePathArray = {
		"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/9252_VRI_Boompjes_constructie.ifc",
		//"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gebouw_rv19_p.v.ifc",
		//"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gevel_rv19_p.v.ifc"
	};

	std::vector<std::string> exportPathArray = {
		"D:/Documents/Uni/Thesis/sources/Models/exports/AC20-FZK-Haus.ifc"
	};

	std::vector<helper*> hFiles;

	// initialize helper
	for (size_t i = 0; i < sourcePathArray.size(); i++)
	{
		std::cout << "Parsing file " << sourcePathArray[i] << std::endl;
		helper* h = new helper(sourcePathArray[i]);

		if (!h->hasSetUnits()) { return 0; }

		hFiles.emplace_back(h);
	}

	// evaluate the storeys
	for (size_t i = 0; i < hFiles.size(); i++)
	{
		std::vector<double> storeyElevation = floorProcessor::getStoreyElevations(hFiles[i]);

		std::vector<std::vector<double>> floorElevation = floorProcessor::getFloorElevations(hFiles[i]);

		// create new storeys if needed
		if (!floorProcessor::compareElevations(storeyElevation, floorElevation))
		{
			std::cout << "found storeys:" << std::endl;
			floorProcessor::printLevels(storeyElevation);

			std::cout << "computed storeys:" << std::endl;
			floorProcessor::printLevels(floorElevation);

			// TODO wipe storeys
			// TODO create new storeys
			// TODO match objects to new storeys
		}

		
	}
	//TODO clash detection
	

	IfcSchema::IfcBuildingStorey* newFloor = new IfcSchema::IfcBuildingStorey(
		guid(),						// GlobalID
		0,							//OwnerHistory
		std::string("floor"),		//Name
		boost::none,				// Description
		boost::none,				// ObjectType
		0,							// ObjectPlacement
		0,							// Representation
		boost::none,				// Tag
		IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT, // Composition type
		0
	);

	

	// create working basefile
	IfcHierarchyHelper<IfcSchema> workingFile;
	workingFile.header().file_name().name("test.ifc");

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