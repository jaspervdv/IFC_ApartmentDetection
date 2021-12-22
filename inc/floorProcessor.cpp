#include "floorProcessor.h"

std::vector<TopoDS_Face> floorProcessor::getSlabFaces(helper* data) {

	std::vector<TopoDS_Face> floorFaces;
	IfcSchema::IfcSlab::list::ptr slabs = data->getSourceFile()->instances_by_type<IfcSchema::IfcSlab>();

	auto kernel = data->getKernel();

	for (IfcSchema::IfcSlab::list::it it = slabs->begin(); it != slabs->end(); ++it) {
		const IfcSchema::IfcSlab* slab = *it;

		// get the IfcShapeRepresentation
		auto slabProduct = slab->Representation()->Representations();

		//get the global coordinate of the local origin
		gp_Trsf trsf;
		kernel->convert_placement(slab->ObjectPlacement(), trsf);

		for (auto et = slabProduct.get()->begin(); et != slabProduct.get()->end(); et++) {
			const IfcSchema::IfcRepresentation* slabRepresentation = *et;

			// select the body of the slabs (ignore the bounding boxes)
			if (slabRepresentation->data().getArgument(1)->toString() == "'Body'")
			{
				// select the geometry format
				auto slabItems = slabRepresentation->Items();

				IfcSchema::IfcRepresentationItem* slabItem = *slabRepresentation->Items().get()->begin();

				auto ob = kernel->convert(slabItem);

				// move to OpenCASCADE
				const TopoDS_Shape rShape = ob[0].Shape();
				const TopoDS_Shape aShape = rShape.Moved(trsf); // location in global space

				// set variables for top face selection
				TopoDS_Face topFace;
				double topHeight = -9999;

				// loop through all faces of slab
				TopExp_Explorer expl;
				for (expl.Init(aShape, TopAbs_FACE); expl.More(); expl.Next())
				{
					TopoDS_Face face = TopoDS::Face(expl.Current());
					BRepAdaptor_Surface brepAdaptorSurface(face, Standard_True);


					// select floor top face
					double faceHeight = face.Location().Transformation().TranslationPart().Z();

					if (faceHeight > topHeight) 
					{ 
						topFace = face; 
						topHeight = faceHeight;
					}
				}
				floorFaces.emplace_back(topFace);
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

std::vector<double> floorProcessor::computeElevations(std::vector<TopoDS_Face> faces) {
	bool debug = true;

	// grouping floor faces by connection
	std::vector<int> pairing(faces.size(), -1);
	std::vector<double> z;
	int group = 0;

	// pair per "pure" elevation
	for (size_t i = 0; i < faces.size(); i++) {

		if (pairing[i] == -1)
		{
			pairing[i] = group;
			group++;
		}

		TopoDS_Face rfloorFace = faces[i];
		double height = rfloorFace.Location().Transformation().TranslationPart().Z();
		z.emplace_back(height);

		for (size_t j = 0; j < faces.size(); j++)
		{
			if (j <= i) { continue; }
			if (faces[j].Location().Transformation().TranslationPart().Z() == height)
			{
				pairing[j] = pairing[i];
			}
		}
	}

	bool consolidated = false;
	int iteration = 0;
	std::vector<double> finalElevationList;

	double t = 1;
	int maxIterations = 1;

	while (!consolidated)
	{
		// find smallest difference between "pure" elevations
		std::vector<double> currentElevations;
		for (size_t i = 0; i < group; i++)
		{
			for (size_t j = 0; j < pairing.size(); j++)
			{
				if (pairing[j] == i) {
					currentElevations.emplace_back(z[j]);
					break;
				}
			}
		}

		double smallestDiff = 9000;
		std::tuple<int, int> smallDistanceIndex;
		for (size_t i = 0; i < currentElevations.size(); i++)
		{
			for (size_t j = i; j < currentElevations.size(); j++)
			{
				if (i == j)
				{
					continue;
				}
				double distance = sqrt(pow(currentElevations[j] - currentElevations[i], 2));

				if (distance < smallestDiff)
				{
					smallestDiff = distance;
				}
			}
		}

		//std::cout << smallestDiff << std::endl;

		if (smallestDiff > t)
		{
			finalElevationList = currentElevations;
			break;
		}
		else if (iteration > maxIterations) {
			std::cout << "[Warning] Consolidation of storeys has failed" << std::endl;
			break;
		}

		// collect area of the storeys
		std::vector<double> areaList = floorProcessor::getFaceAreas(faces);
		std::vector<double> storeyAreaList(group, -1);

		for (size_t i = 0; i < group; i++)
		{
			for (size_t j = 0; j < pairing.size(); j++)
			{
				if (pairing[j] == i)
				{
					if (storeyAreaList[i] == -1) { storeyAreaList[i] = areaList[j]; }
					else { storeyAreaList[i] = storeyAreaList[i] + areaList[j]; }
				}
			}
		}

		//TODO merge small differences?

		iteration++;
	}

	// TODO merge neighbours (not flat surfaces)
	for (size_t i = 0; i < faces.size(); i++) {
		
		TopoDS_Face face1 = faces[i];		
		TopExp_Explorer expl;
		std::vector<std::tuple<gp_Pnt, gp_Pnt, double>> pairedDistance; // contains 2 points and the distance between those two points
		std::vector<gp_Pnt> pointList;

		// get all vertex from a face
		for (expl.Init(face1, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			pointList.emplace_back(BRep_Tool::Pnt(vertex));
		}

		// pair the vertex to create edges and compute distances
		int maxidx = pointList.size();
		for (size_t j = 0; j < maxidx; j += 2)
		{
			gp_Pnt p1 = pointList[j];
			gp_Pnt p2 = pointList[j + 1];

			double distance = p1.Distance(p2);
			pairedDistance.emplace_back(std::make_tuple(p1, p2, distance));		
		}

		for (size_t j = 0; j < faces.size(); j++)
		{
			if (pairing[i] == pairing[j]) { continue; }

			bool neighbouring = false;

			TopoDS_Face face2 = faces[j];

			// get all vertex from a face
			for (expl.Init(face2, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p3 = BRep_Tool::Pnt(vertex);

				for (size_t k = 0; k < pairedDistance.size(); k++)
				{
					auto pair = pairedDistance[k];
					auto referenceDistance = std::get<2>(pair);

					double computedDistance = p3.Distance(std::get<0>(pair)) + p3.Distance(std::get<1>(pair));
					if (computedDistance + 0.05 > referenceDistance && computedDistance - 0.05 < referenceDistance) {

						//std::cout << computedDistance << " -- " << referenceDistance << std::endl;
						neighbouring = true;

						//pairing[j] = pairing[i];

						for (size_t l = 0; l < pairing.size(); l++)
						{
							if (pairing[l] == pairing[j])
							{
								pairing[l] = pairing[i];
							}
						}
					}
					if (neighbouring) { break; }
				}
			}
		}
	}

	if (debug) {
		for (size_t i = 0; i < faces.size(); i++)
		{
			std::cout << faces[i].Location().Transformation().TranslationPart().Z() << " - " << pairing[i] << std::endl;
		}
	}

	//get matching height
	double maxCount = *std::max_element(pairing.begin(), pairing.end());
	std::vector<double> computedElev;
	int count = 0;
	bool allfound = false;

	while (!allfound)
	{
		for (size_t i = 0; i < z.size(); i++)
		{
			//std::cout << i << " = " << count << std::endl;
			if (pairing[i] == count)
			{
				computedElev.emplace_back(z[i]);
				break;
			}
			
		}
		count++;
		if (count > maxCount) {	break; }
	}

	//std::cout << "size: " << computedElev.size() << std::endl;

	if (debug)
	{

		for (size_t i = 0; i < computedElev.size(); i++)
		{
			std::cout << "computedElev: " << computedElev[i] << std::endl;
		}
	}
	return computedElev;

}


void floorProcessor::printLevels(std::vector<double> levels) {
	std::sort(levels.begin(), levels.end());
	for (unsigned int i = 0; i < levels.size(); i++)
	{
		std::cout << levels[i] << std::endl;
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

std::vector<double> floorProcessor::getFloorElevations(helper* data)
{
	// get the floor faces and floor areas
	std::vector<TopoDS_Face> floorFaces = floorProcessor::getSlabFaces(data);
	std::vector<double> floorArea = floorProcessor::getFaceAreas(floorFaces);

	// find the biggest slab face area
	double bigArea = 0;
	for (size_t i = 0; i < floorArea.size(); i++)
	{
		std::cout << floorArea[i] << std::endl;

		if (floorArea[i] > bigArea) {
			bigArea = floorArea[i];
		}
	}

	//TODO scale to correct unit;

	std::cout << "B: " << bigArea << std::endl;

	// ignore small slabs form the process
	double q = 0.0;

	std::vector<TopoDS_Face> filteredFaces;
	for (size_t i = 0; i < floorFaces.size(); i++)
	{
		if (floorArea[i] > bigArea * q)
		{
			filteredFaces.emplace_back(floorFaces[i]);
		}
	}

	std::vector<double> Elevations = floorProcessor::computeElevations(filteredFaces);
	std::cout << "test: " << Elevations.size() << std::endl;
	return Elevations;
}

bool floorProcessor::compareElevations(std::vector<double> elevations, std::vector<double> floors)
{
	bool sameSize = false;
	if (floors.size() == elevations.size())
	{
		std::cout << "[Info] Detected floors and storeys match" << std::endl;
		return true;
	}
	else
	{
		std::cout << "[Info] Detected floors and storeys mismatch!" << std::endl;
		std::cout << "- " << floors.size() << " floors detected, " << elevations.size() << " storeys placed." << std::endl;
		return false;
	}

	for (size_t i = 0; i < elevations.size(); i++)
	{
		double rightElevation = floors[i];
		double leftElevation = elevations[i];

	}
	return true;
}

void floorProcessor::cleanStoreys(helper* data)
{
	// remove the storey object
	IfcSchema::IfcBuildingStorey::list::ptr oldStoreys = data->getSourceFile()->instances_by_type<IfcSchema::IfcBuildingStorey>();

	for (IfcSchema::IfcBuildingStorey::list::it it = oldStoreys->begin(); it != oldStoreys->end(); ++it)
	{
		IfcSchema::IfcBuildingStorey* storey = *it;
		data->getSourceFile()->removeEntity(storey);
	}

	// remove the storey container
	IfcSchema::IfcRelContainedInSpatialStructure::list::ptr containers = data->getSourceFile()->instances_by_type<IfcSchema::IfcRelContainedInSpatialStructure>();

	for (IfcSchema::IfcRelContainedInSpatialStructure::list::it it = containers->begin(); it != containers->end(); ++it)
	{
		IfcSchema::IfcRelContainedInSpatialStructure* container = *it;
		data->getSourceFile()->removeEntity(container);
	}

}

void floorProcessor::createStoreys(helper* data, std::vector<double> floorStoreys)
{
	IfcSchema::IfcOwnerHistory* ownerHistory = data->getHistory();

	// find original owner history
	IfcSchema::IfcBuilding* building;
	IfcSchema::IfcBuilding::list::ptr buildings = data->getSourceFile()->instances_by_type<IfcSchema::IfcBuilding>();
	
	if (buildings.get()->size() != 1)
	{
		std::cout << "[Error] multiple building objects found!" << std::endl;
		return;
	}

	building = *buildings.get()->begin();

	auto targetFile = data->getSourceFile();
	IfcHierarchyHelper<IfcSchema> hierarchyHelper;

	for (size_t i = 0; i < floorStoreys.size(); i++)
	{
		// create storey objects
		auto storey = hierarchyHelper.addBuildingStorey(building, ownerHistory);
		storey->setElevation(floorStoreys[i]);
		storey->setName("Floor");
		storey->setDescription("Automatically generated floor");

		IfcSchema::IfcProduct::list::ptr parts(new IfcSchema::IfcProduct::list);

		// make container object
		IfcSchema::IfcRelContainedInSpatialStructure* container = new IfcSchema::IfcRelContainedInSpatialStructure(
			IfcParse::IfcGlobalId(),		// GlobalId
			0,								// OwnerHistory
			std::string(""),				// Name
			boost::none,					// Description
			parts,							// Related Elements
			storey							// Related structure
		);
		hierarchyHelper.addEntity(container);
	}

	// add storey objects to the project
	for (auto it = hierarchyHelper.begin(); it != hierarchyHelper.end(); ++it)
	{
		auto hierarchyElement = *it;
		auto hierarchyDataElement = hierarchyElement.second;
		auto objectName = hierarchyDataElement->declaration().name();

		// remove potential dublications made by the helper
		// TODO remove all dependencies of building and owners as well
		if (objectName == "IfcBuilding" || objectName == "IfcOwnerHistory") { continue; }

		targetFile->addEntity(hierarchyDataElement);
	}

}



void floorProcessor::sortObjects(helper* data)
{
	IfcParse::IfcFile*  sourcefile = data->getSourceFile();
	auto kernel = data->getKernel();

	// make a vector with the height, spatial structure and a temp product  list
	IfcSchema::IfcRelContainedInSpatialStructure::list::ptr containers = sourcefile->instances_by_type<IfcSchema::IfcRelContainedInSpatialStructure>();

	std::vector<std::tuple<double, IfcSchema::IfcRelContainedInSpatialStructure*, IfcSchema::IfcProduct::list::ptr>> pairedContainers;

	for (auto it= containers->begin(); it != containers->end(); ++it)
	{
		IfcSchema::IfcRelContainedInSpatialStructure* structure = *it;
		IfcSchema::IfcProduct::list::ptr container(new IfcSchema::IfcProduct::list);
		double height = std::stod(structure->RelatingStructure()->data().getArgument(9)->toString()) * data->getUnits()[0];
		
		pairedContainers.emplace_back(
			std::make_tuple( 
				height,			// height
				structure,		// spatial structure
				container		// ifcproduct list
			)																												
		);
	}

	std::sort(pairedContainers.begin(), pairedContainers.end());

	// get the elevation of all product in the model
	IfcSchema::IfcProduct::list::ptr products = sourcefile->instances_by_type<IfcSchema::IfcProduct>();

	for (auto it = products->begin(); it != products->end(); ++it)
	{
		IfcSchema::IfcProduct* product = *it;

		if (!product->hasRepresentation()) { continue; }
		if (product->data().type()->name() == "IfcSite") { continue; }

		bool heightFound = false;

		auto representations = product->Representation()->Representations();

		gp_Trsf trsf;
		kernel->convert_placement(product->ObjectPlacement(), trsf);

		double height;

		// floors are a special case due to them being placed based on their top elevation
		
		bool hasBBox = false;

		if (product->data().type()->name() == "IfcSlab")
		{
			for (auto et = representations.get()->begin(); et != representations.get()->end(); et++) {
				const IfcSchema::IfcRepresentation* representation = *et;

				// TODO might fail with angled slabs
				if (representation->data().getArgument(1)->toString() == "'Body'")
				{
					IfcSchema::IfcRepresentationItem* representationItems = *representation->Items().get()->begin();

					auto ob = kernel->convert(representationItems);

					// move to OpenCASCADE
					const TopoDS_Shape rShape = ob[0].Shape();
					const TopoDS_Shape aShape = rShape.Moved(trsf); // location in global space

					// set variables for top face selection
					TopoDS_Face topFace;
					double topHeight = -9999;

					// loop through all faces of slab
					TopExp_Explorer expl;
					for (expl.Init(aShape, TopAbs_FACE); expl.More(); expl.Next())
					{
						TopoDS_Face face = TopoDS::Face(expl.Current());
						// select floor top face
						double faceHeight = face.Location().Transformation().TranslationPart().Z();

						if (faceHeight > topHeight) { topHeight = faceHeight; }
					}
					height = topHeight;
					break;
				}
			}
		}
		else {
			for (auto et = representations.get()->begin(); et != representations.get()->end(); et++) {
				const IfcSchema::IfcRepresentation* representation = *et;

				// select the bounding box of the objects
				// the easy way of getting the placement height
				if (representation->data().getArgument(2)->toString() == "'BoundingBox'") {
					hasBBox = true;;

					auto items = representation->Items();

					if (items.get()->size() != 1)
					{
						std::cout << "[Warning] more than one item has been found at object with id: " << product->data().id() << std::endl;
					}

					IfcSchema::IfcRepresentationItem* item = *items.get()->begin();
					std::string x = item->get("Corner")->toString();
					x.erase(0, 1);

					IfcUtil::IfcBaseClass* point = sourcefile->instance_by_id(std::stoi(x));

					// get the height of the object
					height = std::stod(point->data().getArgument(0)[0][2]->toString()) + trsf.TranslationPart().Z();
					break;
				}
			}

			// The challenging way of getting the placement height
			if (!hasBBox)
			{

				for (auto et = representations.get()->begin(); et != representations.get()->end(); et++) {
					const IfcSchema::IfcRepresentation* representation = *et;

					std::string geotype = representation->data().getArgument(2)->toString();
					if (representation->data().getArgument(1)->toString() != "'Body'") { continue; }

					if (geotype == "'Brep'" ||
						geotype == "'SweptSolid'" ||
						geotype == "'MappedRepresentation'" ||
						geotype == "'Clipping'") {

						IfcSchema::IfcRepresentationItem* representationItems = *representation->Items().get()->begin();
						auto ob = kernel->convert(representationItems);

						// move to OpenCASCADE
						const TopoDS_Shape rShape = ob[0].Shape();
						const TopoDS_Shape aShape = rShape.Moved(trsf); // location in global space

						// set variables for top face selection
						TopoDS_Face topFace;
						double lowHeight = 9999;

						// loop through all faces of slab
						TopExp_Explorer expl;
						for (expl.Init(aShape, TopAbs_FACE); expl.More(); expl.Next())
						{
							TopoDS_Face face = TopoDS::Face(expl.Current());
							// select floor top face
							double faceHeight = face.Location().Transformation().TranslationPart().Z();

							if (faceHeight < lowHeight) { lowHeight = faceHeight; }
						}
						height = lowHeight;
						break;
					}
				}
			}
		}

		

		int maxidx = pairedContainers.size();

		for (size_t i = 0; i < maxidx; i++)
		{
			auto currentTuple = pairedContainers[i];
			double distance = height - std::get<0>(currentTuple);

			if (distance == 0)
			{
				std::get<2>(currentTuple)->push(product);
				break;
			}
			if (i + 1 == maxidx)
			{
				std::get<2>(currentTuple)->push(product);
				break;
			}

			double nextDistance = height - std::get<0>(pairedContainers[i + 1]);

			if (distance > 0 && nextDistance < 0)
			{
				std::get<2>(currentTuple)->push(product);
				break;
			}
		}

		//std::cout << std::endl;
	}

	for (size_t i = 0; i < pairedContainers.size(); i++)
	{
		auto currentTuple = pairedContainers[i];

		std::get<1>(currentTuple)->setRelatedElements(std::get<2>(currentTuple));
	}

}

