#include "floorProcessor.h"

std::vector<TopoDS_Face> floorProcessor::getSlabFaces(helper* data) {

	std::vector<TopoDS_Face> floorFaces;
	IfcSchema::IfcSlab::list::ptr slabs = data->getSourceFile()->instances_by_type<IfcSchema::IfcSlab>();

	for (IfcSchema::IfcSlab::list::it it = slabs->begin(); it != slabs->end(); ++it) {
		const IfcSchema::IfcSlab* slab = *it;

		// get the IfcShapeRepresentation
		auto slabProduct = slab->Representation()->Representations();

		// can be removed in future
		//std::cout << slab->data().toString() << std::endl;

		//get the global coordinate of the local origin
		gp_Trsf trsf;
		data->getKernel()->convert_placement(slab->ObjectPlacement(), trsf);

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

					auto ob = data->getKernel()->convert(slabItem);

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

						if (faceHeight > topHeight) { topFace = face; }
					}
					floorFaces.emplace_back(topFace);
				}
			}
		}
	}
	return floorFaces;
}

std::vector<double> floorProcessor::getFaceAreas(std::vector<TopoDS_Face> faces) {

	std::vector<double> floorArea;

	for (size_t i = 0; i < faces.size(); i++)
	{
		//get area of topface
		GProp_GProps gprops;
		BRepGProp::SurfaceProperties(faces[i], gprops); // Stores results in gprops
		double area = gprops.Mass();

		floorArea.emplace_back(area);
	}

	return floorArea;
}

std::vector<int> floorProcessor::getPairing(std::vector<TopoDS_Face> faces) {

	// grouping floor faces by connection
	bool sorted = false;
	std::vector<int> pairing(faces.size(), -1);
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
		}

		sorted = true;
		for (size_t i = 0; i < faces.size(); i++)
		{

			// face has not been given a group yet
			if (pairing[i] == -1)
			{
				pairing[i] = group;
				group++;
			}

			TopoDS_Face rfloorFace = faces[i];
			std::vector<gp_Pnt> floorPoints;

			TopExp_Explorer expl;
			for (expl.Init(rfloorFace, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				floorPoints.emplace_back(BRep_Tool::Pnt(vertex));
			}

			for (size_t j = 0; j < faces.size(); j++)
			{
				if (i == j) { continue; }
				if (pairing[i] == pairing[j] && pairing[i] != -1) { continue; }

				TopoDS_Face lfloorFace = faces[j];

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

	return pairing;
}


std::vector<std::vector<double>> floorProcessor::getLevel(std::vector<TopoDS_Face> faces, std::vector<int> pairs)
{
	double allowedDistance = 0.05;

	// find the amount of groups that have been created
	int groupsize = 0;
	for (size_t i = 0; i < pairs.size(); i++)
	{
		if (pairs[i] > groupsize)
		{
			groupsize = pairs[i];
		}
	}

	// pair the elevations to the found groups
	std::vector<double> topElevation(groupsize + 1, 9999);
	for (size_t i = 0; i < faces.size(); i++)
	{
		TopoDS_Face floorFace = faces[i];
		double xyz_z = 9999;

		TopExp_Explorer expl;
		for (expl.Init(floorFace, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt evalPoint = BRep_Tool::Pnt(vertex);
			if (evalPoint.Z() < xyz_z) { xyz_z = evalPoint.Z(); }
		}
		if (topElevation[pairs[i]] > xyz_z) { topElevation[pairs[i]] = xyz_z; }
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

	// merge levels that are very close to each other
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

	return groupedFilteredTopElevation;
}

void floorProcessor::printLevels(std::vector<double> levels) {
	std::sort(levels.begin(), levels.end());

	std::cout << "storey elevations: " << std::endl;
	for (unsigned int i = 0; i < levels.size(); i++)
	{
		std::cout << levels[i] << std::endl;
	}
}

void floorProcessor::printLevels(std::vector<std::vector<double>> levels) {
	std::sort(levels.begin(), levels.end());

	std::cout << "floor elevations: " << std::endl;
	for (size_t i = 0; i < levels.size(); i++)
	{
		std::cout << "{";
		for (size_t j = 0; j < levels[i].size(); j++)
		{
			std::cout << levels[i][j] << std::endl;
		}
		std::cout << "}" << std::endl;
	}
}


std::vector<double> floorProcessor::getStoreyElevations(helper* data)
{
	IfcSchema::IfcBuildingStorey::list::ptr storeys = data->getSourceFile()->instances_by_type<IfcSchema::IfcBuildingStorey>();
	std::vector<double> storeyElevation;

	for (IfcSchema::IfcBuildingStorey::list::it it = storeys->begin(); it != storeys->end(); ++it)
	{
		const IfcSchema::IfcBuildingStorey* storey = *it;
		storeyElevation.emplace_back(storey->Elevation() * data->getLengthMultiplier());
	}

	if (!storeyElevation.size()) { std::cout << "No storeys can be found" << std::endl; }

	return storeyElevation;
}

std::vector<std::vector<double>> floorProcessor::getFloorElevations(helper* data)
{
	// get the floor faces and floor areas
	std::vector<TopoDS_Face> floorFaces = floorProcessor::getSlabFaces(data);
	std::vector<double> floorArea = floorProcessor::getFaceAreas(floorFaces);

	// find the biggest slab face area
	double bigArea = 0;
	for (size_t i = 0; i < floorArea.size(); i++)
	{
		if (floorArea[i] > bigArea) {
			bigArea = floorArea[i];
		}
	}

	// ignore small slabs form the process
	double q = 0.2;

	std::vector<TopoDS_Face> filteredFaces;
	for (size_t i = 0; i < floorFaces.size(); i++)
	{
		if (floorArea[i] > bigArea * q)
		{
			filteredFaces.emplace_back(floorFaces[i]);
		}
	}

	std::vector<int> pairing = floorProcessor::getPairing(filteredFaces);
	std::vector<std::vector<double>> elevations = floorProcessor::getLevel(filteredFaces, pairing);

	return elevations;
}

bool floorProcessor::compareElevations(std::vector<double> elevations, std::vector<std::vector<double>> floors)
{
	bool sameSize = false;
	if (floors.size() == elevations.size())
	{
		std::cout << "[Info] Detected floors and storeys match" << std::endl;
		sameSize = true;
	}
	else
	{
		std::cout << "[Warning] Detected floors and storeys mismatch!" << std::endl;
		std::cout << "- " << floors.size() << " floors detected, " << elevations.size() << " storeys placed." << std::endl;
		std::cout << "- This might be caused by complex floor and roof shapes, process is continued with the placed storeys" << std::endl;
		return false;
	}

	for (size_t i = 0; i < elevations.size(); i++)
	{
		std::vector<double> rightElevation = floors[i];
		double leftElevation = elevations[i];

		if (rightElevation.size() == 1)
		{
			if (rightElevation[0] != leftElevation) {
				std::cout << "[Info] elevation mismatch" << std::endl;
				std::cout << "- This might be caused by complex floor and roof shapes, process is continued with the placed storeys" << std::endl;
				return false;
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
				std::cout << "[Info] elevation mismatch" << std::endl;
				std::cout << "- This might be caused by complex floor and roof shapes, process is continued with the placed storeys" << std::endl;
				return false;
			}
		}
	}
	return true;
}
