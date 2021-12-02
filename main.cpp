// TODO: Multiple schemas
#define IfcSchema Ifc2x3

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

	//find schema of file
	std::ifstream infile(sourcePath);
	std::string line;
	while (std::getline(infile, line))
	{
		if (line[0] == '#') {
			break;
		}

		if (line.find("FILE_SCHEMA(('IFC4'))") != std::string::npos) {
			std::cout << "Valid scheme" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC2X3'))") != std::string::npos) {
			//TODO translate IFC2x3 to IFC4
			break;
		}

	}
	infile.close();

	// get source file
	IfcParse::IfcFile* SourceFile = new IfcParse::IfcFile(sourcePath);

	if (!SourceFile->good()) {
		std::cout << "Unable to parse .ifc file" << std::endl;
		return 1;
	}
	else {
		std::cout << "Valid IFC file found" << std::endl;
	}
	
	std::cout << std::endl;

	// create working basefile
	IfcHierarchyHelper<IfcSchema> workingFile;
	workingFile.header().file_name().name("test.ifc");

	IfcGeom::Kernel my_kernel(SourceFile);

	// check if unit assignment is valid
	IfcSchema::IfcUnitAssignment::list::ptr presentUnits = SourceFile->instances_by_type<IfcSchema::IfcUnitAssignment>();
	if (presentUnits.get()->size() == 0) {
		std::cout << "[Error] No unit assignment has been found" << std::endl;
		return 0;
	}
	else if (presentUnits.get()->size() > 1)
	{
		std::cout << "[Error] Multiple unit assignments have been found" << std::endl;
		return 0;
	}
	
	// get the unit
	double lengthMultiplier = 0;
	double areaMultiplier = 0;
	double volumeMultiplier = 0;

	for (IfcSchema::IfcUnitAssignment::list::it it = presentUnits->begin(); it != presentUnits->end(); ++it)
	{
		const IfcSchema::IfcUnitAssignment* itUnits = *it;
		auto units = itUnits->Units();
		
		for (auto et = units.get()->begin(); et != units.get()->end(); et++) {
			auto unit = *et;

			if (unit->declaration().type() == 902 || unit->declaration().type() == 765) // select the IfcSIUnit
			{
				std::string unitType = unit->data().getArgument(1)->toString();
				std::string SiUnitBase = unit->data().getArgument(3)->toString();
				std::string SiUnitAdd = unit->data().getArgument(2)->toString();

				if (unitType == ".LENGTHUNIT.")
				{
					if (SiUnitBase == ".METRE.") { lengthMultiplier = 1; }
					if (SiUnitAdd == ".MILLI.") { lengthMultiplier = lengthMultiplier/1000; }
				}
				else if (unitType == ".AREAUNIT.")
				{
					if (SiUnitBase == ".SQUARE_METRE.") { areaMultiplier = 1; }
					if (SiUnitAdd == ".MILLI.") { areaMultiplier = areaMultiplier / pow(1000,2); }
				}
				if (unitType == ".VOLUMEUNIT.")
				{
					if (SiUnitBase == ".CUBIC_METRE.") { volumeMultiplier = 1; }
					if (SiUnitAdd == ".MILLI.") { volumeMultiplier = volumeMultiplier / pow(1000, 3); }
				}
			}
		}
	}

	// check if units have been found
	std::cout << "found units:" << std::endl;
	if (!lengthMultiplier)
	{
		std::cout << "[Error] SI unit for length cannot be found!" << std::endl;
		return 0;
	}
	else if (lengthMultiplier == 1) { std::cout << "- Lenght in metre" << std::endl; }
	else if (lengthMultiplier == 0.001) { std::cout << "- Lenght in millimetre" << std::endl; }

	if (!areaMultiplier)
	{
		std::cout << "[Error] SI unit for area cannot be found!" << std::endl;
		return 0;
	}
	else if (areaMultiplier == 1) { std::cout << "- Area in square metre" << std::endl; }
	else if (areaMultiplier == 0.000001) { std::cout << "- Area in square millimetre" << std::endl; }

	if (!volumeMultiplier)
	{
		std::cout << "[Error] SI unit for volume cannot be found!" << std::endl;
		return 0;
	}
	else if (volumeMultiplier == 1) { std::cout << "- Volume in cubic metre" << std::endl; }
	else if (volumeMultiplier == 0.000000001) { std::cout << "- Volume in cubic millimetre" << std::endl; }

	std::cout << std::endl;

	IfcSchema::IfcBuildingElement::list::ptr elements = SourceFile->instances_by_type<IfcSchema::IfcBuildingElement>();
	std::cout << "Found " << elements->size() << " elements " << std::endl;

	IfcSchema::IfcBuildingStorey::list::ptr storeys = SourceFile->instances_by_type<IfcSchema::IfcBuildingStorey>();
	std::vector<double> floorArea;
	std::vector<double> storeyElevation;
	std::vector<double> floorElevation;
	std::vector<TopoDS_Face> floorFaces;

	for (IfcSchema::IfcBuildingStorey::list::it it = storeys->begin(); it != storeys->end(); ++it)
	{
		const IfcSchema::IfcBuildingStorey* storey = *it;
		storeyElevation.emplace_back(storey->Elevation() * lengthMultiplier);
	}

	if (!storeyElevation.size()) {std::cout << "No storeys can be found" << std::endl;}

	IfcSchema::IfcSlab::list::ptr slabs = SourceFile->instances_by_type<IfcSchema::IfcSlab>();

	double bigArea = 0;

	for (IfcSchema::IfcSlab::list::it it = slabs->begin(); it != slabs->end(); ++it) {
		const IfcSchema::IfcSlab* slab = *it;

		// get the IfcShapeRepresentation
		auto slabProduct = slab->Representation()->Representations();

		// can be removed in future
		//std::cout << slab->data().toString() << std::endl;

		//get the global coordinate of the local origin
		gp_Trsf trsf;
		my_kernel.convert_placement(slab->ObjectPlacement(), trsf);

		for (auto et = slabProduct.get()->begin(); et != slabProduct.get()->end(); et++) {
			const IfcSchema::IfcRepresentation* slabRepresentation = *et;

			// select the body of the slabs (ignore the bounding boxes)
			if (slabRepresentation->data().getArgument(1)->toString() == "'Body'")
			{
				// select the geometry format
				auto slabItems = slabRepresentation->Items();

				for (auto st = slabItems.get()->begin(); st != slabItems.get()->end(); st++)
				{
					IfcSchema::IfcRepresentationItem* slabItem = *st;

					auto ob = my_kernel.convert(slabItem);

					// move to OpenCASCADE
					const TopoDS_Shape rShape = ob[0].Shape();
					const TopoDS_Shape aShape = rShape.Moved(trsf); // location in global space

					// set variables for top face selection
					TopoDS_Face topFace;
					double topHeight = -9999;
					double buttomHeight = 0;
					int faceCount = 0;

					// loop through all faces of slab
					TopExp_Explorer expl;
					for (expl.Init(aShape, TopAbs_FACE); expl.More(); expl.Next())
					{
						faceCount++;
						TopoDS_Face face = TopoDS::Face(expl.Current());
						BRepAdaptor_Surface brepAdaptorSurface(face, Standard_True);


						// select floor top face
						double faceHeight = face.Location().Transformation().TranslationPart().Z();

						if (faceHeight > topHeight){topFace = face;}
					}

					//get area of topface
					GProp_GProps gprops;
					BRepGProp::SurfaceProperties(topFace, gprops); // Stores results in gprops
					double area = gprops.Mass();

					// get biggest present slab area
					if (area > bigArea) { bigArea = area; }

					floorArea.emplace_back(area);
					floorFaces.emplace_back(topFace);
				}
			}
		}
	}

	// remove floorslabs for evaluation if they are smaller than 20% of the biggest present slab
	std::vector<TopoDS_Face> filteredFaces;
	for (size_t i = 0; i < floorFaces.size(); i++)
	{
		if (floorArea[i] > bigArea * 0.2)
		{
			filteredFaces.emplace_back(floorFaces[i]);
		}
	}

	// grouping floor faces by connection
	bool sorted = false;
	std::vector<int> pairing(filteredFaces.size(), -1);
	int group = 0;
	int maxIterations = 1000; //TODO scale based on amount of present slabs
	int curIteration = 0;
	double allowedDistance = 0.05;

	while (!sorted)
	{
		if (curIteration == maxIterations)
		{
			// TODO return error but continue with the process
			std::cout << "[Warning] Floorslab matching has been unsuccesful " << std::endl;
			return 0;
		}

		sorted = true;
		for (size_t i = 0; i < filteredFaces.size(); i++)
		{

			// face has not been given a group yet
			if (pairing[i] == -1)
			{
				pairing[i] = group;
				group++;
			}

			TopoDS_Face rfloorFace = filteredFaces[i];
			std::vector<gp_Pnt> floorPoints;

			TopExp_Explorer expl;
			for (expl.Init(rfloorFace, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				floorPoints.emplace_back(BRep_Tool::Pnt(vertex));
			}

			for (size_t j = 0; j < filteredFaces.size(); j++)
			{
				if (i == j) { continue; }
				if (pairing[i] == pairing[j] && pairing[i] != -1) { continue; }

				TopoDS_Face lfloorFace = filteredFaces[j];

				for (expl.Init(lfloorFace, TopAbs_VERTEX); expl.More(); expl.Next())
				{
					TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
					gp_Pnt pt = BRep_Tool::Pnt(vertex);

					for (size_t k = 0; k < floorPoints.size(); k++)
					{
						gp_Pnt evalPoint = floorPoints[k];

						// look for neighbours
						// TODO change to allow neighbours that are not connected with cornerpoints
						if (pt.X() > evalPoint.X() - allowedDistance && pt.X() < evalPoint.X() + allowedDistance && 
							pt.Y() > evalPoint.Y() - allowedDistance && pt.Y() < evalPoint.Y() + allowedDistance &&
							pt.Z() > evalPoint.Z() - allowedDistance && pt.Z() < evalPoint.Z() + allowedDistance)
						{
							int base = pairing[j];
							int change = pairing[i];

							for (size_t l = 0; l < pairing.size(); l++)
							{
								if (pairing[l] == base && pairing[l] != -1) { pairing[l] = change; }
							}
							pairing[j] = pairing[i];
							sorted = false; // not finished sorting if a neighbour is found
							continue;
						}

					}

				}
			}

		}
		curIteration++;
	}

	// correct possible holes in the pairing list
	group = group - 1;
	int stableGroup = group;

	for (size_t i = 0; i < stableGroup; i++)
	{
		auto it = std::find(pairing.begin(), pairing.end(), i);

		if (it != pairing.end()) { continue; }

		int minDistance = 9999;
		for (size_t j = 0; j < pairing.size(); j++)
		{
			if (pairing[j] > i)
			{
				int distance = pairing[j] - i;
				if (distance < minDistance) { minDistance = distance; }
			}
		}
		for (size_t j = 0; j < pairing.size(); j++)
		{
			if (pairing[j] > i) { pairing[j] = pairing[j] - minDistance; }
		}
		if (minDistance < 9999) { group = group - minDistance; }
	}

	std::vector<double> topElevation(group, 9999);
	for (size_t i = 0; i < filteredFaces.size(); i++)
	{
		TopoDS_Face floorFace = filteredFaces[i];
		double xyz_z = 9999;

		TopExp_Explorer expl;
		for (expl.Init(floorFace, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt evalPoint = BRep_Tool::Pnt(vertex);
			if (evalPoint.Z() < xyz_z) { xyz_z = evalPoint.Z(); }
		}
		if (topElevation[pairing[i]] > xyz_z) { topElevation[pairing[i]] = xyz_z; }
	}
	
	// remove dublicates 
	std::vector<double> filteredTopElevation;
	for (size_t i = 0; i < topElevation.size(); i++)
	{
		bool dub = false;

		if (!filteredTopElevation.size())
		{
			filteredTopElevation.emplace_back(topElevation[i]);
		}
		else 
		{
			for (size_t j = 0; j < filteredTopElevation.size(); j++)
			{
				if (filteredTopElevation[j] < topElevation[i] + allowedDistance && filteredTopElevation[j] > topElevation[i] - allowedDistance)
				{
					dub = true;
					continue;
				} 
			}

			if (!dub) { filteredTopElevation.emplace_back(topElevation[i]); }
		}
	}

	// merge small increments of floors
	double maxDistance = 1.5;
	int groupnr = 0;
	std::vector<int> grouping(filteredTopElevation.size(), -1);
	for (size_t i = 0; i < filteredTopElevation.size(); i++)
	{
		if (grouping[i] == -1)
		{
			grouping[i] = groupnr;
			groupnr++;
		}

		for (size_t j = 0; j < filteredTopElevation.size(); j++)
		{
			if (j > i) { continue; }
			double distance = sqrt(pow(filteredTopElevation[i] - filteredTopElevation[j], 2));
			if (distance < maxDistance)
			{
				int base = grouping[j];
				for (size_t k = 0; k < filteredTopElevation.size(); k++)
				{
					if (grouping[k] == base) {
						grouping[k] = grouping[i];
					}
				}
			}
		}
	}

	// merge dublicates
	std::vector<std::vector<double>> groupedFilteredTopElevation;
	for (size_t i = 0; i < groupnr; i++)
	{
		std::vector<double> tempfloor;
		for (size_t j = 0; j < filteredTopElevation.size(); j++)
		{
			if (grouping[j] == i)
			{
				tempfloor.emplace_back(filteredTopElevation[j]);
			}
		}

		if (tempfloor.size())
		{
			groupedFilteredTopElevation.emplace_back(tempfloor);
		}
	}

	//return warning if failed

	bool sameSize = false;
	if (groupedFilteredTopElevation.size() == storeyElevation.size())
	{
		std::cout << "[Info] Detected floors and storeys match" << std::endl;
		sameSize = true;
	}
	else 
	{
		std::cout << "[Warning] Detected floors and storeys mismatch!" << std::endl;
		std::cout << "- " << groupedFilteredTopElevation.size() << " floors detected, " << storeyElevation.size() << " storeys placed." << std::endl;
		std::cout << "- This might be caused by complex floor and roof shapes, process is continued with the placed storeys" << std::endl;
	}


	if (sameSize)
	{
		for (size_t i = 0; i < storeyElevation.size(); i++)
		{
			std::vector<double> rightElevation = groupedFilteredTopElevation[i];
			double leftElevation = storeyElevation[i];

			if (rightElevation.size() == 1)
			{
				if (rightElevation[0] != leftElevation) {
					std::cout << "[Warning] elevation mismatch" << std::endl;
					std::cout << "- This might be caused by complex floor and roof shapes, process is continued with the placed storeys" << std::endl;
					break;
				}
			}
			else
			{
				double hMin = 9999;
				double hMax = -9999;
				for (size_t j = 0; j < rightElevation.size(); j++)
				{
					if (rightElevation[j] < hMin)
					{
						hMin = rightElevation[j];
					}

					if (rightElevation[j] > hMax)
					{
						hMax = rightElevation[j];
					}
				}

				if (leftElevation > hMax || leftElevation < hMin) 
				{
					std::cout << "[Warning] elevation mismatch" << std::endl;
					std::cout << "- This might be caused by complex floor and roof shapes, process is continued with the placed storeys" << std::endl;
					break;
				}
			}
		}

	}

	std::sort(storeyElevation.begin(), storeyElevation.end());
	std::sort(filteredTopElevation.begin(), filteredTopElevation.end());
	/*
	std::cout << "storey elevations: " << std::endl;
	for (unsigned int i = 0; i < storeyElevation.size(); i++)
	{
		std::cout << storeyElevation[i] << std::endl;
	} 

	std::cout << "detected elevations: " << std::endl;
	for (size_t i = 0; i < filteredTopElevation.size(); i++)
	{
		std::cout << filteredTopElevation[i] << std::endl;
	}
	*/
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