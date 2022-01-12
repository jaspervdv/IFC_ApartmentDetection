#include "floorProcessor.h"

double floorProcessor::getMedian(std::vector<double> l)
{
	assert(!l.empty());
	if (l.size() % 2 == 0) {
		const auto median_it1 = l.begin() + l.size() / 2 - 1;
		const auto median_it2 = l.begin() + l.size() / 2;

		std::nth_element(l.begin(), median_it1, l.end());
		const auto e1 = *median_it1;

		std::nth_element(l.begin(), median_it2, l.end());
		const auto e2 = *median_it2;

		return (e1 + e2) / 2;

	}
	else {
		const auto median_it = l.begin() + l.size() / 2;
		std::nth_element(l.begin(), median_it, l.end());
		return *median_it;
	}
}

std::vector<TopoDS_Shape> floorProcessor::getSlabShapes(helper* data)
{
	std::vector<TopoDS_Shape> floorShapes;
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

				floorShapes.emplace_back(aShape);
			}
		}
	}

	return floorShapes;

}

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

std::vector<TopoDS_Face> floorProcessor::getSlabFaces(std::vector<TopoDS_Shape> shapes)
{
	std::vector<TopoDS_Face> floorFaces;
	for (size_t i = 0; i < shapes.size(); i++)
	{
		TopoDS_Shape floorshape = shapes[i];

		// set variables for top face selection
		TopoDS_Face topFace;
		double topHeight = -9999;

		// loop through all faces of slab
		TopExp_Explorer expl;
		for (expl.Init(floorshape, TopAbs_FACE); expl.More(); expl.Next())
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

void floorProcessor::updateFloorGroup(std::vector<FloorGroupStruct>* floorGroups)
{
	std::vector<int> removeIdx;

	int idx = 0;
	for (auto it = floorGroups->begin(); it != floorGroups->end(); it++)
	{
		floorProcessor::FloorGroupStruct currentGroup = *it;

		if (currentGroup.mergeNum_ == 1)
		{
			currentGroup.merger_->mergeGroup(&currentGroup);
			removeIdx.emplace_back(idx);
		}
		idx++;
	}
	std::reverse(removeIdx.begin(), removeIdx.end());

	for (size_t i = 0; i < removeIdx.size(); i++)
	{
		floorGroups->erase(floorGroups->begin() + removeIdx[i]);
	}
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
	std::sort(storeyElevation.begin(), storeyElevation.end());

	return storeyElevation;
}

std::vector<double> floorProcessor::getFloorElevations(helper* data)
{
	// get the top faces of the floors
	std::vector< TopoDS_Shape> floorShapes = floorProcessor::getSlabShapes(data);
	std::vector<TopoDS_Face> floorFaces = floorProcessor::getSlabFaces(floorShapes);

	// do not compute elevation if the model is partial and not a construction model 
	if (data->getDepending() && !data->getIsConstruct()) { return {};	}

	if (!data->getDepending())
	{
		// TODO detect floor thickness
		// TODO get orientating bounding box (based on the longest edge)
		// TODO get shortest side of the bounding box
		// TODO make a rule with the width and the orentated bounding box
	}

	bool debug = false;

	// make floor struct
	std::vector<double> faceAreas = floorProcessor::getFaceAreas(floorFaces);

	auto SmallestAllowedArea = getMedian(faceAreas) * 0.1;

	std::vector<FloorStruct> floorList;
	for (size_t i = 0; i < floorFaces.size(); i++)
	{
		// filter out small floor slabs
		//if (faceAreas[i] > SmallestAllowedArea)
		{
			floorProcessor::FloorStruct floorobject(floorFaces[i], faceAreas[i]);
			floorList.emplace_back(floorobject);
		}
	}



	std::vector<FloorGroupStruct> floorGroups;

	// pair per "pure" elevation
	for (size_t i = 0; i < floorList.size(); i++) {

		floorProcessor::FloorGroupStruct floorGroup;

		if (!floorList[i].hasGroup)
		{
			floorGroup.addFloor(&floorList[i]);
			floorList[i].hasGroup = true;

			double height = floorList[i].elevation_;

			for (size_t j = 0; j < floorList.size(); j++)
			{
				if (floorList[j].hasGroup) { continue; }
				double otherHeight = floorList[j].elevation_;

				if (otherHeight + 0.000001 > height && otherHeight - 0.000001 < height)
				{
					floorGroup.addFloor(&floorList[j]);
					floorList[j].hasGroup = true;
				}
			}

		}
		if (floorGroup.floors_.size() > 0) { floorGroups.emplace_back(floorGroup); }
	}

	// find neighbours
	for (size_t i = 0; i < floorGroups.size(); i++)
	{
		auto currentGroup = &floorGroups[i];

		// get a face from a group
		for (size_t j = 0; j < currentGroup->floors_.size(); j++)
		{
			TopoDS_Face face1 = currentGroup->floors_[j]->face_;
			TopExp_Explorer expl;
			std::vector<DistancePair> pairedDistance;
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

				floorProcessor::DistancePair pair;
				pair.p1 = p1;
				pair.p2 = p2;
				pair.distance = p1.Distance(p2);;

				pairedDistance.emplace_back(pair);
			}

			for (size_t k = i + 1; k < floorGroups.size(); k++)
			{
				bool found = false;
				auto matchingGroup = &floorGroups[k];

				// get a face from a group to match
				for (size_t l = 0; l < matchingGroup->floors_.size(); l++)
				{

					TopoDS_Face face2 = matchingGroup->floors_[l]->face_;

					for (expl.Init(face2, TopAbs_VERTEX); expl.More(); expl.Next())
					{
						TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
						gp_Pnt p3 = BRep_Tool::Pnt(vertex);

						for (size_t m = 0; m < pairedDistance.size(); m++)
						{
							auto& pair = pairedDistance[m];
							auto referenceDistance = pair.distance;

							double computedDistance = p3.Distance(pair.p1) + p3.Distance(pair.p2);
							if (computedDistance + 0.05 > referenceDistance && computedDistance - 0.05 < referenceDistance) {
								if (matchingGroup->mergeNum_ == -1) {
									matchingGroup->merger_ = currentGroup;
									matchingGroup->mergeNum_ = 1;
								}
								else {
									for (size_t n = 0; n < floorGroups.size(); n++)
									{
										if (floorGroups[n].merger_ == matchingGroup)
										{
											floorGroups[n].merger_ = currentGroup;
										}
										matchingGroup->merger_ = currentGroup;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	// merge the groups
	updateFloorGroup(&floorGroups);

	// find overlapping buffers
	for (size_t i = 0; i < floorGroups.size(); i++)
	{
		auto currentGroup = &floorGroups[i];
		for (size_t j = i + 1; j < floorGroups.size(); j++)
		{
			auto matchingGroup = &floorGroups[j];

			if (currentGroup->elevation_ > matchingGroup->elevation_ && currentGroup->topElevation_ < matchingGroup->topElevation_ ||
				currentGroup->elevation_ < matchingGroup->elevation_ && currentGroup->topElevation_ > matchingGroup->topElevation_ ||
				currentGroup->elevation_ < matchingGroup->topElevation_ && currentGroup->topElevation_ > matchingGroup->topElevation_ ||
				currentGroup->elevation_ < matchingGroup->elevation_ && currentGroup->topElevation_ > matchingGroup->elevation_
				)
			{
				if (matchingGroup->mergeNum_ == -1) {
					matchingGroup->merger_ = currentGroup;
					matchingGroup->mergeNum_ = 1;
				}

				else {
					for (size_t k = 0; k < floorGroups.size(); k++)
					{
						if (floorGroups[k].merger_ == matchingGroup)
						{
							floorGroups[k].merger_ = currentGroup;
						}
						matchingGroup->merger_ = currentGroup;
					}
				}

			}

		}
	}

	// merge the groups
	updateFloorGroup(&floorGroups);

	// find small elevation differences
	double maxDistance = 1.f;

	for (size_t i = 0; i < floorGroups.size(); i++)
	{
		auto currentGroup = &floorGroups[i];
		for (size_t j = i + 1; j < floorGroups.size(); j++)
		{
			auto matchingGroup = &floorGroups[j];
			if (std::abs(currentGroup->elevation_ - matchingGroup->elevation_) < maxDistance)
			{
				// always merge to the floor with the smallest elevation
				auto targetGroup = currentGroup;
				auto sourceGroup = matchingGroup;

				if (targetGroup->elevation_ > sourceGroup->elevation_)
				{
					targetGroup = matchingGroup;
					sourceGroup = currentGroup;
				}

				if (sourceGroup->mergeNum_ == -1) {
					sourceGroup->merger_ = targetGroup;
					sourceGroup->mergeNum_ = 1;
				}
				else 
				{
					for (size_t k = 0; k < floorGroups.size(); k++)
					{
						if (floorGroups[k].merger_ == sourceGroup) { floorGroups[k].merger_ = targetGroup; }
						sourceGroup->merger_ = targetGroup;
					}
				}
			}
		}
	}

	// merge the groups
	updateFloorGroup(&floorGroups);

	std::vector<double> computedElev;
	for (size_t i = 0; i < floorGroups.size(); i++) { computedElev.emplace_back(floorGroups[i].elevation_); }


	if (debug)
	{
		for (size_t i = 0; i < computedElev.size(); i++)
		{
			std::cout << "computedElev: " << computedElev[i] << std::endl;
		}
	}

	std::sort(computedElev.begin(), computedElev.end());

	return computedElev;
}

bool floorProcessor::compareElevations(std::vector<double> elevations, std::vector<double> floors)
{
	bool sameSize = false;
	if (floors.size() == elevations.size())
	{
		for (size_t i = 0; i < elevations.size(); i++)
		{
			double rightElevation = floors[i];
			double leftElevation = elevations[i];

			if (rightElevation != leftElevation) { return false; }

		}
		return true;
	}
	else
	{
		//std::cout << "- " << floors.size() << " floors detected, " << elevations.size() << " storeys placed." << std::endl;
		return false;
	}
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
	double lengthMulti = data->getUnits()[0];

	// make a vector with the height, spatial structure and a temp product  list
	IfcSchema::IfcRelContainedInSpatialStructure::list::ptr containers = sourcefile->instances_by_type<IfcSchema::IfcRelContainedInSpatialStructure>();

	std::vector<std::tuple<double, IfcSchema::IfcRelContainedInSpatialStructure*, IfcSchema::IfcProduct::list::ptr>> pairedContainers;

	for (auto it= containers->begin(); it != containers->end(); ++it)
	{
		IfcSchema::IfcRelContainedInSpatialStructure* structure = *it;
		IfcSchema::IfcProduct::list::ptr container(new IfcSchema::IfcProduct::list);
		double height = std::stod(structure->RelatingStructure()->data().getArgument(9)->toString()) * lengthMulti;
		
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

		// vind smallest distance to floor elevation
		double smallestDistance = 1000;
		double indxSmallestDistance = 0;
		for (size_t i = 0; i < maxidx; i++) {
			auto currentTuple = pairedContainers[i];
			double distance = height * lengthMulti - std::get<0>(currentTuple);
			
			if (distance < 0) { break; }

			if (distance < smallestDistance)
			{
				smallestDistance = distance;
				indxSmallestDistance = i;
			} 

			if (distance == 0) { break; }
		}

		std::get<2>(pairedContainers[indxSmallestDistance])->push(product);
	}

	for (size_t i = 0; i < pairedContainers.size(); i++)
	{
		auto currentTuple = pairedContainers[i];
		std::get<1>(currentTuple)->setRelatedElements(std::get<2>(currentTuple));
	}
}

floorProcessor::FloorStruct::FloorStruct(TopoDS_Face face, double area)
{
	face_ = face;
	hasFace = true;

	isFlat_ = true;
	hasFlatness = true;

	// get the elevation and flatness
	double lowHeight = -9999;
	double topHeight = -9999;
	TopExp_Explorer expl;

	for (expl.Init(face, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt point = BRep_Tool::Pnt(vertex);

		double currentHeight = point.Z();

		if (lowHeight == -9999) { lowHeight = currentHeight; }
		else if (lowHeight != currentHeight)
		{
			isFlat_ = false;
			if (lowHeight > currentHeight) { lowHeight = currentHeight; }
		}

		if (topHeight == -9999) { topHeight = currentHeight; }
		else if (topHeight != currentHeight)
		{
			if (topHeight < currentHeight) { topHeight = currentHeight; }
		}


	}
	elevation_ = lowHeight;
	elevation_ = lowHeight;
	topElevation_ = topHeight;
	hasElevation = true;
}

floorProcessor::FloorGroupStruct::FloorGroupStruct(FloorStruct* floor)
{
	assigned_ = true;
	isFlat_ = floor->isFlat_;
	topElevation_ = floor->elevation_;
	elevation_ = floor->elevation_;

	floors_.emplace_back(floor);
}

void floorProcessor::FloorGroupStruct::addFloor(FloorStruct* floor)
{
	assigned_ = true;
	if (isFlat_) { isFlat_ = floor->isFlat_; }
	if (topElevation_ < floor->topElevation_) { topElevation_ = floor->topElevation_; }
	if (elevation_ > floor->elevation_) { elevation_ = floor->elevation_; }

	floors_.emplace_back(floor);
}

void floorProcessor::FloorGroupStruct::mergeGroup(FloorGroupStruct* group)
{
	for (size_t i = 0; i < group->floors_.size(); i++) { floors_.emplace_back(group->floors_[i]); }
	if (isFlat_) { isFlat_ = group->isFlat_; }
	if (topElevation_ < group->topElevation_) { topElevation_ = group->topElevation_; }
	if (elevation_ > group->elevation_) { elevation_ = group->elevation_; }
}
