#include "inc/helper.h"
#include "inc/floorProcessor.h"
#include "inc/roomProcessor.h"

// basic includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <chrono>

// boost includes
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/algorithm/string.hpp>

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

std::vector<std::string> GetSources() {

	// TODO replace with file open prompt
	std::vector<std::string> sourcePathArray = {
	//"D:/Documents/Uni/Thesis/sources/Models/simple_models/box_60.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/simple_models/simple_rooms.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/simple_models/curtainWall_1.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/simple_models/wall_curtain_1.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/simple_models/RoomInRoom.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/simple_models/covex_concave.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/simple_models/rotatedRoomTest.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/On4/Stramien hoogte.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/Revit_Example_Models/rac_basic_sample_project.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/Revit_Example_Models/FM_ARC_DigitalHub.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/AC-20-Smiley-West-10-Bldg.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/AC20-Institute-Var-2.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/AC20-FZK-Haus.ifc"
	"D:/Documents/Uni/Thesis/sources/Models/exports/Exported_AC-20-Smiley-West-10-Bldg.ifc"
	//"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/9252_VRI_Boompjes_constructie.ifc",
	//"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gebouw_rv19_p.v.ifc",
	//"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gevel_rv19_p.v.ifc"
	};

	return sourcePathArray;
}

void compareElevationsOutput(const std::vector<double> left, const std::vector<double> right) {
	int tab = 35;
	if (left.size() != right.size())
	{
		if (left.size() > right.size())
		{
			int minIndx = (int) right.size();
			for (size_t i = 0; i < left.size(); i++)
			{
				if (i < minIndx) { std::cout << std::left << std::setw(tab) << left[i] << right[i] << std::endl; }
				else { std::cout << std::left << std::setw(tab) << left[i] << "-" << std::endl; }
			}
		}
		else {
			int minIndx = (int) left.size();
			for (size_t i = 0; i < right.size(); i++)
			{
				if (i < minIndx) { std::cout << std::left << std::setw(tab) << left[i] << right[i] << std::endl; }
				else { std::cout << std::left << std::setw(tab) << "-" << right[i] << std::endl; }
			}
		}
	}
	else
	{
		for (size_t i = 0; i < right.size(); i++) { std::cout << std::left << std::setw(tab) << left[i] << right[i] << std::endl; }
	}
}

bool yesNoQuestion() {
	std::string cont = "";

	while (true)
	{
		std::cin >> cont;

		if (cont == "Y" || cont == "y") { return true; }
		if (cont == "N" || cont == "n") { return false; }
	}
}


int main(int argc, char** argv) {



	// outputs errors related to the selected objects
	if (false) { Logger::SetOutput(&std::cout, &std::cout); }

	std::vector<std::string> sourcePathArray = GetSources();

	// make export path
	std::vector<std::string> exportPathArray;
	std::vector<std::string> fileNames;
	for (size_t i = 0; i < sourcePathArray.size(); i++)
	{
		std::string exportPath;
		std::vector<std::string> segments;
		boost::split(segments, sourcePathArray[i], boost::is_any_of("/"));

		for (size_t i = 0; i < segments.size()-1; i++) { exportPath += segments[i] + "/"; }

		fileNames.emplace_back(segments[segments.size() - 1]);
		exportPath += "exports/Exported_" + segments[segments.size() - 1];
		exportPathArray.emplace_back(exportPath);
	}



	int constructionIndx = -1;

	// get construction model num from user
	if (sourcePathArray.size() == 1)
	{
		std::cout << "[INFO] One file found, considered combination file" << std::endl;
	}
	else {
		while (true)
		{
			bool validInput = true;
			std::string stringNum = "";

			std::cout << "Please enter number of construction model, if no constuction model enter 0." << std::endl;
			for (size_t i = 0; i < fileNames.size(); i++) { std::cout << i + 1 << ": " << fileNames[i] << std::endl; }
			std::cout << "Num: ";
			std::cin >> stringNum;

			for (size_t i = 0; i < stringNum.size(); i++)
			{
				if (!std::isdigit(stringNum[i]))
				{
					validInput = false;
				}
			}

			if (validInput)
			{
				constructionIndx = std::stoi(stringNum) - 1;
				if (fileNames.size() >= constructionIndx + 1) {
					break;
				}
			}

			std::cout << "\n [INFO] Please enter a valid number! \n" << std::endl;
		}
	}

	std::cout << std::endl;

	// initialize helper
	helperCluster* hCluster = new helperCluster;

	for (size_t i = 0; i < sourcePathArray.size(); i++)
	{
		std::cout << "Parsing file " << sourcePathArray[i] << std::endl;
		helper* h = new helper(sourcePathArray[i]);
		
		if (sourcePathArray.size() > 1) { 
			h->setDepending(true); 

			// set the construction model
			if (i == constructionIndx) { h->setIsConstruct(true); }
		}
		else { h->setIsConstruct(true); }
		if (!h->hasSetUnits()) { return 0; }
		h->setName(fileNames[i]);		

		hCluster->appendHelper(h);
	}

	hCluster->internaliseData();

	// evaluate the storeys
	std::vector<double> floorElevation = floorProcessor::computeFloorElevations(hCluster->getHelpers());
	std::vector<double> StoredFloorElevation = floorProcessor::getStoreyElevations(hCluster->getHelpers());

	// allow user to select which elevations are used
	std::vector<double> usedElevations = StoredFloorElevation;
	if (!floorProcessor::compareElevations(StoredFloorElevation, floorElevation))
	{
		std::cout << "[INFO] Software detected different storey elevations than are stored in the IFC file \n" << std::endl;
		std::cout << std::left << std::setw(35) << "IFC file storey elevations" << "Computed Elevations" << std::endl;
		compareElevationsOutput(StoredFloorElevation, floorElevation);
		std::cout << "\nUse the recalculated Storey elevations? (Y/N): ";

		if (yesNoQuestion()) { usedElevations = floorElevation; }
	}
	else {
		std::cout << "[INFO] Software detected identical storey elevations as are stored in the IFC file" << std::endl;
	}

	if (usedElevations == StoredFloorElevation)
	{
		std::cout << "\nContinue with sorting process?\nQuality of the results may vary\n(Y/N): ";

		if (yesNoQuestion()) 
		{ 
			// create new storeys and sort all object into them
			std::cout << std::endl; 
			floorProcessor::processStoreys(hCluster->getHelpers(), usedElevations);
		}

	}
	else {
		std::cout << std::endl;
		floorProcessor::processStoreys(hCluster->getHelpers(), usedElevations);
	}

	std::cout << std::endl;

	if (sourcePathArray.size() != 1)
	{
		while (true)
		{
			bool validInput = true;
			std::string stringNum = "";

			std::cout << "Please enter number of the target model for the spaces/rooms" << std::endl;
			for (size_t i = 0; i < fileNames.size(); i++) { std::cout << i + 1 << ": " << fileNames[i] << std::endl; }
			std::cout << "Num: ";
			std::cin >> stringNum;

			for (size_t i = 0; i < stringNum.size(); i++)
			{
				if (!std::isdigit(stringNum[i]))
				{
					validInput = false;
				}
			}

			if (validInput)
			{
				int roomIndx = std::stoi(stringNum) - 1;

				if (roomIndx >= 0) { 
					hCluster->getHelper(roomIndx)->setHasRooms();
					break;
				}
			}
			std::cout << "\n [INFO] Please enter a valid number! \n" << std::endl;
		}
	}
	else {
		hCluster->getHelper(0)->setHasRooms();
	}

	for (int i = 0; i < hCluster->getSize(); i++)
	{
		hCluster->getHelper(i)->indexGeo();
		hCluster->getHelper(i)->correctRooms();
	}
	auto startTime = std::chrono::high_resolution_clock::now();

	voxelfield* field = new voxelfield(hCluster);


	field->makeRooms(hCluster);



	//TODO room detection and check
	//TODO room creation
	//TODO appartement detection
	//TODO appartement creation

	// write to file

	for (int i = 0; i < hCluster->getSize(); i++)
	{
		hCluster->getHelper(i)->writeToFile(exportPathArray[i]);
	}

	std::cout << "last line executed" << std::endl;

	auto endTime = std::chrono::high_resolution_clock::now();

	std::cout << "computing time = " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count() << std::endl;
	
	return 0;

}