// TODO: Multiple schemas
#define IfcSchema Ifc4

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

	std::cout << "[INFO] This build is made for " << IfcSchema::get_schema().name() << std::endl;

	// TODO replace with file open prompt
	std::vector<std::string> sourcePathArray = {
		"D:/Documents/Uni/Thesis/sources/Models/AC20-FZK-Haus.ifc"
		//"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/9252_VRI_Boompjes_constructie.ifc"//,
		//"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gebouw_rv19_p.v.ifc",
		//"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gevel_rv19_p.v.ifc"
	};

	// TODO replace with file save prompt
	// make export path
	std::vector<std::string> exportPathArray;
	for (size_t i = 0; i < sourcePathArray.size(); i++)
	{
		std::string exportPath;
		std::vector<std::string> segments;
		boost::split(segments, sourcePathArray[i], boost::is_any_of("/"));

		for (size_t i = 0; i < segments.size()-1; i++) { exportPath += segments[i] + "/"; }
		exportPath += "exports/Exported_" + segments[segments.size() - 1];
		exportPathArray.emplace_back(exportPath);
	}

	bool findElevations = true;

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

		if (findElevations)
		{
			std::vector<double> floorElevation = floorProcessor::getFloorElevations(hFiles[i]);

			// create new storeys if needed
			if (!floorProcessor::compareElevations(storeyElevation, floorElevation))
			{
				std::cout << "found storeys:" << std::endl;
				floorProcessor::printLevels(storeyElevation);

				std::cout << "computed storeys:" << std::endl;
				floorProcessor::printLevels(floorElevation);

				// wipe storeys
				floorProcessor::cleanStoreys(hFiles[i]);
				// create new storeys
				floorProcessor::createStoreys(hFiles[i], floorElevation);
			}
		}

		// match objects to new storeys
		floorProcessor::sortObjects(hFiles[i]);
	}
	//TODO clash detection

	// write to file
	hFiles[0]->writeToFile(exportPathArray[0]);

	std::cout << "last line executed" << std::endl;
	
	return 0;

}