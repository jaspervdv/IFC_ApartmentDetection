// TODO: Multiple schemas
#define IfcSchema Ifc2x3

#include "inc/helper.h"
#include "inc/floorProcessor.h"
#include "inc/roomProcessor.h"

// basic includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>

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

typedef IfcParse::IfcGlobalId guid;

std::vector<std::string> GetSources() {

	// TODO replace with file open prompt
	std::vector<std::string> sourcePathArray = {
	//"D:/Documents/Uni/Thesis/sources/Models/AC20-FZK-Haus.ifc"
	"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/9252_VRI_Boompjes_constructie.ifc",
	"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gebouw_rv19_p.v.ifc",
	"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gevel_rv19_p.v.ifc"
	};

	return sourcePathArray;
}

void compareElevationsOutput(const std::vector<double> left, const std::vector<double> right) {
	int tab = 35;
	if (left.size() != right.size())
	{
		if (left.size() > right.size())
		{
			int minIndx = right.size();
			for (size_t i = 0; i < left.size(); i++)
			{
				if (i < minIndx) { std::cout << std::left << std::setw(tab) << left[i] << right[i] << std::endl; }
				else { std::cout << std::left << std::setw(tab) << left[i] << "-" << std::endl; }
			}
		}
		else {
			int minIndx = left.size();
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

	std::cout << "[INFO] This build is made for " << IfcSchema::get_schema().name() << std::endl;

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
		std::cout << "one file found, considered combination file" << std::endl;
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

	std::vector<helper*> hFiles;

	// initialize helper
	for (size_t i = 0; i < sourcePathArray.size(); i++)
	{
		std::cout << "Parsing file " << sourcePathArray[i] << std::endl;
		helper* h = new helper(sourcePathArray[i]);
		
		if (sourcePathArray.size() > 1) { h->setDepending(true); }
		if (!h->hasSetUnits()) { return 0; }

		// set the construction model
		if (i == constructionIndx) { h->setIsConstruct(true); }
		h->setName(fileNames[i]);		

		hFiles.emplace_back(h);
	}

	// evaluate the storeys
	std::vector<double> floorElevation = floorProcessor::computeFloorElevations(hFiles);
	std::vector<double> StoredFloorElevation = floorProcessor::getStoreyElevations(hFiles);

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
		std::cout << "\nContinue with sorting process?(Y/N): ";

		if (yesNoQuestion()) { std::cout << std::endl; floorProcessor::processStoreys(hFiles, usedElevations); }

	}
	std::cout << std::endl;

	// create new storeys and sort all object into them
	

	// create new rooms

	//TODO room detection and check
	//TODO room creation
	//TODO appartement detection
	//TODO appartement creation

	// write to file

	for (size_t i = 0; i < hFiles.size(); i++)
	{
		hFiles[i]->writeToFile(exportPathArray[i]);
	}

	std::cout << "last line executed" << std::endl;
	
	return 0;

}