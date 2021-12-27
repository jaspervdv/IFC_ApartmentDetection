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

int main(int argc, char** argv) {

	Logger::SetOutput(&std::cout, &std::cout);

	std::cout << "[INFO] This build is made for " << IfcSchema::get_schema().name() << std::endl;

	// TODO replace with file open prompt
	std::vector<std::string> sourcePathArray = {
		//"D:/Documents/Uni/Thesis/sources/Models/AC20-FZK-Haus.ifc"
		"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/9252_VRI_Boompjes_constructie.ifc",
		"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gebouw_rv19_p.v.ifc",
		"D:/Documents/Uni/Thesis/sources/Models/Rotterdam/160035-Boompjes_TVA_gevel_rv19_p.v.ifc"
	};

	// TODO replace with file save prompt
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

	std::vector<int> constructionIndxList;
	// get construction model num from user

	if (sourcePathArray.size() == 1)
	{
		std::cout << "one file found, considered combination file" << std::endl;
	}
	else {

		while (true)
		{
			constructionIndxList.clear();
			bool validInput = true;
			std::string stringNum = "";
			std::string constructionIndx;

			std::cout << "Please enter number of construction model, if multiple numbers seperaty by ',', if no constuction model enter 0." << std::endl;
			for (size_t i = 0; i < fileNames.size(); i++) { std::cout << i + 1 << ": " << fileNames[i] << std::endl; }
			std::cout << "Num: ";
			std::cin >> constructionIndx;

			for (size_t i = 0; i < constructionIndx.size(); i++)
			{
				bool isDig = isdigit(constructionIndx[i]);

				if (!isDig && !constructionIndx.size() - 1 == i)
				{
					std::cout << "Input invalid!" << std::endl;
					validInput = false;
					break;
				}
				else {
					if (constructionIndx[i] == ',' || constructionIndx.size() - 1 == i)
					{
						int inputNum;

						if (constructionIndx.size() - 1 == i && isDig)
						{
							inputNum = std::stoi(stringNum += constructionIndx[i]);
						}
						else {
							inputNum = std::stoi(stringNum);
						}

						stringNum = "";

						if (inputNum <= fileNames.size()) { constructionIndxList.emplace_back(inputNum); }
						else {
							std::cout << "Input invalid!" << std::endl;
							validInput = false;
							break;
						}
					}
					else if (isDig) {
						stringNum += constructionIndx[i];
					}
				}

			}
			if (validInput) {
				std::cout << std::endl;
				for (size_t i = 0; i < constructionIndxList.size(); i++)
				{
					if (constructionIndxList[i] == 0) { std::cout << "No construction model present." << std::endl; }
					else { std::cout << "Model '" << fileNames[constructionIndxList[i] - 1] << "' has been selected as construction model" << std::endl; }
				}

				while (true)
				{
					std::string con;
					std::cout << "continue? (Y/N): ";
					std::cin >> con;

					boost::to_upper(con);

					if (con == "Y") { break; }
					else if (con == "N")
					{
						validInput = false;
						break;
					}
					else { std::cout << "Invalid input" << std::endl; }

				}

			}
			if (validInput) { break; }
		}
	}

	std::cout << std::endl;
	bool findElevations = true;

	std::vector<helper*> hFiles;

	// initialize helper
	for (size_t i = 0; i < sourcePathArray.size(); i++)
	{
		std::cout << "Parsing file " << sourcePathArray[i] << std::endl;
		helper* h = new helper(sourcePathArray[i]);
		
		if (sourcePathArray.size() > 1) { h->setDepending(true); }
		if (!h->hasSetUnits()) { return 0; }

		// set the construction model
		for (size_t j = 0;  j < constructionIndxList.size();  j++)
		{
			if (i == constructionIndxList[j] - 1) {	h->setIsConstruct(true); }
		}
		hFiles.emplace_back(h);
	}

	// evaluate the storeys
	std::vector<double> floorElevation;
	for (size_t i = 0; i < hFiles.size(); i++)
	{
		std::vector<double> storeyElevation = floorProcessor::getStoreyElevations(hFiles[i]);

		if (findElevations)
		{
			std::vector<double> tempFloorElevation = floorProcessor::getFloorElevations(hFiles[i]);

			if (tempFloorElevation.size() != 0)
			{
				floorElevation = tempFloorElevation;

				std::cout << "found storeys:" << std::endl;
				floorProcessor::printLevels(storeyElevation);

				std::cout << "computed storeys:" << std::endl;
				floorProcessor::printLevels(floorElevation);
			}
		}


	}

	for (size_t i = 0; i < hFiles.size(); i++)
	{
		// wipe storeys
		floorProcessor::cleanStoreys(hFiles[i]);
		// create new storeys
		floorProcessor::createStoreys(hFiles[i], floorElevation);
		
		// match objects to new storeys
		floorProcessor::sortObjects(hFiles[i]);

	}

	//TODO clash detection
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