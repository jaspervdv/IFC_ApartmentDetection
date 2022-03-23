#include "helper.h"

void printPoint(gp_Pnt p) {
	std::cout << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << std::endl;
}

void printPoint(BoostPoint3D p) {
	std::cout << bg::get<0>(p) << ", " << bg::get<1>(p) << ", " << bg::get<2>(p) << ", " << std::endl;
}

void printFaces(TopoDS_Shape shape)
{
	//std::cout << "Shape:" << std::endl;
	std::vector<TopoDS_Face> faceList;

	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
	{
		faceList.emplace_back(TopoDS::Face(expl.Current()));
	}

	for (size_t i = 0; i < faceList.size(); i++)
	{
		std::cout << "Start of Face" << std::endl;
		for (expl.Init(faceList[i], TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			printPoint(p);
		}
		std::cout << "End of Face" << std::endl;
		std::cout << std::endl;
	}
	//std::cout << std::endl;
}


gp_Pnt rotatePointWorld(gp_Pnt p, double angle) {
	double pX = p.X();
	double pY = p.Y();
	double pZ = p.Z();

	return gp_Pnt(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}

BoostPoint3D rotatePointWorld(BoostPoint3D p, double angle) {
	double pX = bg::get<0>(p);
	double pY = bg::get<1>(p);
	double pZ = bg::get<2>(p);

	return BoostPoint3D(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}

std::tuple<gp_Pnt, gp_Pnt, double> rotatedBBoxDiagonal(std::vector<gp_Pnt> pointList, double angle) {
	
	bool isPSet = false;
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;

	for (size_t i = 0; i < pointList.size(); i++)
	{
		gp_Pnt point = rotatePointWorld(pointList[i], angle);

		if (!isPSet)
		{
			isPSet = true;

			lllPoint = point;
			urrPoint = point;
		}
		else
		{
			if (point.X() < lllPoint.X()) { lllPoint.SetX(point.X()); }
			if (point.Y() < lllPoint.Y()) { lllPoint.SetY(point.Y()); }
			if (point.Z() < lllPoint.Z()) { lllPoint.SetZ(point.Z()); }

			if (point.X() > urrPoint.X()) { urrPoint.SetX(point.X()); }
			if (point.Y() > urrPoint.Y()) { urrPoint.SetY(point.Y()); }
			if (point.Z() > urrPoint.Z()) { urrPoint.SetZ(point.Z()); }
		}
	}

	// compute diagonal distance
	double distance = lllPoint.Distance(urrPoint);

	return std::make_tuple( lllPoint, urrPoint, distance );
}

BoostPoint3D Point3DOTB(gp_Pnt oP){
	return BoostPoint3D(oP.X(), oP.Y(), oP.Z());
}

gp_Pnt Point3DBTO(BoostPoint3D oP) {
	return gp_Pnt(bg::get<0>(oP), bg::get<1>(oP), bg::get<2>(oP));
}

helper::helper(std::string path) {

	helper::findSchema(path);

	IfcParse::IfcFile*  sourceFile = new IfcParse::IfcFile(path);

	bool good = sourceFile->good();

	if (!good) 
	{ 
		std::cout << "Unable to parse .ifc file" << std::endl; 
	}
	else { 
		std::cout << "Valid IFC file found" << std::endl; 
		std::cout << std::endl;
		
		file_ = sourceFile;

		kernel_ = new IfcGeom::Kernel(file_);

		helper::setUnits(file_);
	}
}


void helper::findSchema(std::string path) {
	std::ifstream infile(path);
	std::string line;

	while (std::getline(infile, line))
	{
		if (line[0] == '#') {
			std::cout << "[Warning] No valid ifc scheme found" << std::endl;
			break; 
		}

		if (line.find("FILE_SCHEMA(('IFC4'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC4" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC2X3'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC2X3" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x1'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC4x1" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x2'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC4x2" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x3'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC4x3" << std::endl;
			break;
		}
	}
	infile.close();
}


std::vector<gp_Pnt> helper::getAllPoints(IfcSchema::IfcProduct::list::ptr products)
{
	std::vector<gp_Pnt> pointList;

	for (auto it = products->begin(); it != products->end(); ++it) {

		IfcSchema::IfcProduct* product = *it;

		if (!product->hasRepresentation()) { continue; }
		if (product->data().type()->name() == "IfcAnnotation") { continue; } // find points another way

		if (product->data().type()->name() == "IfcSite") { continue; }

		std::vector<gp_Pnt> temp = getObjectPoints(product);
		if (temp.size() > 1);
		{
			for (size_t i = 0; i < temp.size(); i++) { pointList.emplace_back(temp[i]); 
			}
		}
		temp.clear();
	}

	return pointList;
}

bool helper::hasSetUnits() {
	if (!length_ || !area_ || !volume_){ return false; }
	else { return true;  }
}


void helper::setUnits(IfcParse::IfcFile *file) {
	double length = 0;
	double area = 0;
	double volume = 0;
	
	IfcSchema::IfcUnitAssignment::list::ptr presentUnits = file->instances_by_type<IfcSchema::IfcUnitAssignment>();
	if (presentUnits.get()->size() == 0) {
		std::cout << "[Error] No unit assignment has been found" << std::endl;
		return;
	}
	else if (presentUnits.get()->size() > 1)
	{
		std::cout << "[Error] Multiple unit assignments have been found" << std::endl;
		return;
	}


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
					if (SiUnitBase == ".METRE.") { length = 1; }
					if (SiUnitAdd == ".MILLI.") { length = length / 1000; }
				}
				else if (unitType == ".AREAUNIT.")
				{
					if (SiUnitBase == ".SQUARE_METRE.") { area = 1; }
					if (SiUnitAdd == ".MILLI.") { area = area / pow(1000, 2); }
				}
				if (unitType == ".VOLUMEUNIT.")
				{
					if (SiUnitBase == ".CUBIC_METRE.") { volume = 1; }
					if (SiUnitAdd == ".MILLI.") { volume = volume / pow(1000, 3); }
				}
			}
		}
	}

	// check if units have been found
	std::cout << "found units:" << std::endl;
	if (!length)
	{
		std::cout << "[Error] SI unit for length cannot be found!" << std::endl;
		return;
	}
	else if (length == 1) { std::cout << "- Lenght in metre" << std::endl; }
	else if (length == 0.001) { std::cout << "- Lenght in millimetre" << std::endl; }

	if (!area)
	{
		std::cout << "[Error] SI unit for area cannot be found!" << std::endl;
		return;
	}
	else if (area == 1) { std::cout << "- Area in square metre" << std::endl; }
	else if (area == 0.000001) { std::cout << "- Area in square millimetre" << std::endl; }

	if (!volume)
	{
		std::cout << "[Error] SI unit for volume cannot be found!" << std::endl;
		return;
	}
	else if (volume == 1) { std::cout << "- Volume in cubic metre" << std::endl; }
	else if (volume == 0.000000001) { std::cout << "- Volume in cubic millimetre" << std::endl; }

	std::cout << std::endl;

	//internalize the data
	length_ = length;
	area_ = area;
	volume_ = volume;
}

void helper::internalizeGeo()
{
	std::cout << "Internalizing Geometry of Construction Model\n" << std::endl;

	// get products
	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();
	std::vector<gp_Pnt> pointList = getAllPoints(products);

	// approximate smalles bbox
	double angle = 22.5 * (M_PI / 180);
	double rotation = 0;
	int maxIt = 15;

	// set data for a bbox
	auto base = rotatedBBoxDiagonal(pointList, rotation);
	gp_Pnt lllPoint = std::get<0>(base);
	gp_Pnt urrPoint = std::get<1>(base);
	double smallestDistance = std::get<2>(base);

	for (size_t i = 0; i < maxIt; i++)
	{
		std::tuple<gp_Pnt, gp_Pnt, double> left;
		std::tuple<gp_Pnt, gp_Pnt, double> right;

		left = rotatedBBoxDiagonal(pointList, rotation - angle);
		right = rotatedBBoxDiagonal(pointList, rotation + angle);
	
		if (std::get<2>(left) > std::get<2>(right) && smallestDistance > std::get<2>(right))
		{ 
			rotation = rotation + angle; 
			smallestDistance = std::get<2>(right);
			lllPoint = std::get<0>(right);
			urrPoint = std::get<1>(right);
		}
		else  if (smallestDistance > std::get<2>(left))
		{ 
			rotation = rotation - angle;
			smallestDistance = std::get<2>(left);
			lllPoint = std::get<0>(left);
			urrPoint = std::get<1>(left);
		}
		
		//make angle smaller
		angle = angle / 2;
	}

	lllPoint_ = lllPoint;
	urrPoint_ = urrPoint;
	originRot_ = rotation;

	std::cout << rotation << std::endl;
	printPoint(lllPoint);
	printPoint(urrPoint_);

	hasGeo = true;
}

void helper::internalizeGeo(double angle) {
	std::cout << "Internalizing Geometry\n" << std::endl;

	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();
	std::vector<gp_Pnt> pointList = getAllPoints(products);

	auto bbox = rotatedBBoxDiagonal(pointList, angle);

	lllPoint_ = std::get<0>(bbox);
	urrPoint_ = std::get<1>(bbox);
	originRot_ = angle;

	hasGeo = true;
}

void helper::indexGeo()
{
	// this indexing is done based on the rotated bboxes of the objects
	// the bbox does thus comply with the model bbox but not with the actual objects original location

	// add the floorslabs to the rtree
	addObjectToIndex<IfcSchema::IfcSlab::list::ptr>(file_->instances_by_type<IfcSchema::IfcSlab>());

	// add the walls to the rtree
	addObjectToIndex<IfcSchema::IfcWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcWall>());
	addObjectToIndex<IfcSchema::IfcCovering::list::ptr>(file_->instances_by_type<IfcSchema::IfcCovering>());

	// add the columns to the rtree TODO sweeps
	addObjectToIndex<IfcSchema::IfcColumn::list::ptr>(file_->instances_by_type<IfcSchema::IfcColumn>());

	// add the beams to the rtree
	addObjectToIndex<IfcSchema::IfcBeam::list::ptr>(file_->instances_by_type<IfcSchema::IfcBeam>());

	// add the curtain walls to the rtree
	addObjectToIndex<IfcSchema::IfcCurtainWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcCurtainWall>());
	addObjectToIndex<IfcSchema::IfcPlate::list::ptr>(file_->instances_by_type<IfcSchema::IfcPlate>());
	addObjectToIndex<IfcSchema::IfcMember::list::ptr>(file_->instances_by_type<IfcSchema::IfcMember>());

	// add doors to the rtree (for the appartment detection)
	addObjectToIndex<IfcSchema::IfcDoor::list::ptr>(file_->instances_by_type<IfcSchema::IfcDoor>());

	// =================================================================================================
	// connictivity objects indexing
	addObjectToCIndex<IfcSchema::IfcDoor::list::ptr>(file_->instances_by_type<IfcSchema::IfcDoor>());
	addObjectToCIndex<IfcSchema::IfcStair::list::ptr>(file_->instances_by_type<IfcSchema::IfcStair>());

	// =================================================================================================
	// room objects indexing
	addObjectToRIndex<IfcSchema::IfcSpace::list::ptr>(file_->instances_by_type<IfcSchema::IfcSpace>());
}

bg::model::box < BoostPoint3D > helper::makeObjectBox(const IfcSchema::IfcProduct* product)
{
	std::vector<gp_Pnt> productVert = getObjectPoints(product);

	if (!productVert.size() > 1) { return bg::model::box < BoostPoint3D >({ 0,0,0 }, { 0,0,0 }); }

	// only outputs 2 corners of the three needed corners!
	auto box = rotatedBBoxDiagonal(productVert, originRot_);

	BoostPoint3D boostlllpoint = Point3DOTB(std::get<0>(box));
	BoostPoint3D boosturrpoint = Point3DOTB(std::get<1>(box));

	return bg::model::box < BoostPoint3D >(boostlllpoint, boosturrpoint);
}

void helper::correctRooms() 
{
	// get all rooms in a cluser
	std::vector<roomLookupValue> roomValues = getFullRLookup();

	// compute volume of every room
	std::vector<double> roomVolume;
	GProp_GProps gprop;

	for (size_t i = 0; i < roomValues.size(); i++)
	{
		BRepGProp::VolumeProperties(std::get<1>(roomValues[i]), gprop);
		roomVolume.emplace_back(gprop.Mass());
		roomCenterPoints_.emplace_back(gprop.CentreOfMass()); // TODO make functioing in con
	}

	// check which room is complex
	for (size_t i = 0; i < roomValues.size(); i++)
	{
		// create bounding box around roomshape
		TopoDS_Shape roomShape = std::get<1>(roomValues[i]);
		std::vector<gp_Pnt> cornerPoints;

		gp_Pnt lll(9999, 9999, 9999);
		gp_Pnt urr(-9999, -9999, -9999);

		TopExp_Explorer expl;
		for (expl.Init(roomShape, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			cornerPoints.emplace_back(p);
		}

		for (size_t j = 0; j < cornerPoints.size(); j++)
		{
			auto currentCorner = cornerPoints[j];
			if (urr.X() < currentCorner.X()) { urr.SetX(currentCorner.X()); }
			if (urr.Y() < currentCorner.Y()) { urr.SetY(currentCorner.Y()); }
			if (urr.Z() < currentCorner.Z()) { urr.SetZ(currentCorner.Z()); }
			if (lll.X() > currentCorner.X()) { lll.SetX(currentCorner.X()); }
			if (lll.Y() > currentCorner.Y()) { lll.SetY(currentCorner.Y()); }
			if (lll.Z() > currentCorner.Z()) { lll.SetZ(currentCorner.Z()); }
		}
		// check if other rooms fall inside evaluating room
		boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(lll), Point3DOTB(urr));

		std::vector<Value> qResult;
		qResult.clear();
		getRoomIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

		if (qResult.size() == 0) { continue; }

		bool complex = false;

		BRepClass3d_SolidClassifier insideChecker;
		insideChecker.Load(roomShape);

		for (size_t j = 0; j < qResult.size(); j++)
		{
			if (i == qResult[j].second) { continue; } // ignore self

			TopoDS_Shape matchingRoom = std::get<1>(roomValues[qResult[j].second]);
			for (expl.Init(matchingRoom, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);
				insideChecker.Perform(p, 0.01);
				if (!insideChecker.State() || insideChecker.IsOnAFace())
				{
					if (roomVolume[i] > roomVolume[qResult[j].second])
					{
						complex = true;
						continue;
					}
				}
				if (complex)
				{
					complex = false;
					break;
				}
			}
			if (complex) {
				std::get<0>(roomValues[i])->setCompositionType(IfcSchema::IfcElementCompositionEnum::IfcElementComposition_COMPLEX);
				break;
			}
		}
		if (!complex)
		{
			std::get<0>(roomValues[i])->setCompositionType(IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT);
			gp_Pnt centerOfMass = gprop.CentreOfMass();
		}
	}
}

std::vector<std::vector<gp_Pnt>> helper::triangulateProduct(IfcSchema::IfcProduct* product)
{
	std::vector<TopoDS_Face> faceList = getObjectFaces(product);
	std::vector<std::vector<gp_Pnt>> triangleMeshList;

	for (size_t i = 0; i < faceList.size(); i++)
	{
		BRepMesh_IncrementalMesh faceMesh = BRepMesh_IncrementalMesh(faceList[i], 0.004);

		TopLoc_Location loc;
		auto mesh = BRep_Tool::Triangulation(faceList[i], loc);
		const gp_Trsf& trsf = loc.Transformation();

		if (mesh.IsNull()) //TODO warning
		{
			continue;
		}

		for (size_t j = 1; j <= mesh.get()->NbTriangles(); j++)
		{
			const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

			std::vector<gp_Pnt> trianglePoints;
			for (size_t k = 1; k <= 3; k++)
			{
				gp_Pnt p = mesh->Nodes().Value(theTriangle(k)).Transformed(trsf);
				trianglePoints.emplace_back(p);
			}
			triangleMeshList.emplace_back(trianglePoints);
		}

	}

	return triangleMeshList;
}

template <typename T>
void helper::addObjectToIndex(T object) {
	// add doors to the rtree (for the appartment detection)
	for (auto it = object->begin(); it != object->end(); ++it) {
		bg::model::box <BoostPoint3D> box = makeObjectBox(*it);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			continue;
		}
		index_.insert(std::make_pair(box, (int) index_.size()));
		std::vector<std::vector<gp_Pnt>> triangleMeshList = triangulateProduct(*it);
		auto lookup = std::make_tuple(*it, triangleMeshList);
		productLookup_.emplace_back(lookup);
	}
}

template <typename T>
void helper::addObjectToCIndex(T object) {
	// add doors to the rtree (for the appartment detection)
	for (auto it = object->begin(); it != object->end(); ++it) {
		bg::model::box <BoostPoint3D> box = makeObjectBox(*it);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			continue;
		}
		cIndex_.insert(std::make_pair(box, (int)cIndex_.size()));
		std::vector<roomObject*>* space = new std::vector<roomObject*>;
		auto lookup = std::make_tuple(*it, space);
		connectivityLookup_.emplace_back(lookup);
	}
}

template<typename T>
void helper::addObjectToRIndex(T object){
	// add doors to the rtree (for the appartment detection)
	for (auto it = object->begin(); it != object->end(); ++it) {
		bg::model::box <BoostPoint3D> box = makeObjectBox(*it);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			continue;
		}
		rIndex_.insert(std::make_pair(box, (int)rIndex_.size()));
		TopoDS_Shape spaceShape = getObjectShape(*it);
		auto lookup = std::make_tuple(*it, spaceShape);
		roomLookup_.emplace_back(lookup);
	}
}



IfcSchema::IfcOwnerHistory* helper::getHistory()
{
	IfcSchema::IfcOwnerHistory* ownerHistory;
	IfcSchema::IfcOwnerHistory::list::ptr ownerHistories = file_->instances_by_type<IfcSchema::IfcOwnerHistory>();

	if (ownerHistories.get()->size() != 1) { std::cout << "[Warning] multiple owners objects found!" << std::endl; }

	return *ownerHistories.get()->begin();
}

std::vector<gp_Pnt> helper::getObjectPoints(const IfcSchema::IfcProduct* product, bool sortEdges)
{
	std::vector<gp_Pnt> pointList;

	if (!product->hasRepresentation()) { return { gp_Pnt(0.0, 0.0, 0.0) }; }

	auto representations = product->Representation()->Representations();

	gp_Trsf trsf;
	kernel_->convert_placement(product->ObjectPlacement(), trsf);

	TopoDS_Shape rShape = getObjectShape(product);

	TopExp_Explorer expl;
	for (expl.Init(rShape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		pointList.emplace_back(p);
	}

	if (!sortEdges) { return pointList; }

	std::vector<gp_Pnt> pointListSmall;
	for (size_t i = 0; i < pointList.size(); i++)
	{
		if (i%2 == 0) { pointListSmall.emplace_back(pointList[i]); }
	}

	return pointListSmall;

}

std::vector<TopoDS_Face> helper::getObjectFaces(const IfcSchema::IfcProduct* product)
{
	std::vector<TopoDS_Face> faceList;

	if (!product->hasRepresentation()) { return {}; }

	TopoDS_Shape rShape = getObjectShape(product);

	TopExp_Explorer expl;
	for (expl.Init(rShape, TopAbs_FACE); expl.More(); expl.Next())
	{
		TopoDS_Face face = TopoDS::Face(expl.Current());
		faceList.emplace_back(face);
	}

	return faceList;
}

TopoDS_Shape helper::getObjectShape(const IfcSchema::IfcProduct* product)
{
	if (!product->hasRepresentation()) { return {}; }

	BRep_Builder builder;
	TopoDS_Compound comp;
	builder.MakeCompound(comp);

	auto representations = product->Representation()->Representations();

	gp_Trsf trsf;
	kernel_->convert_placement(product->ObjectPlacement(), trsf);

	for (auto et = representations.get()->begin(); et != representations.get()->end(); et++) {


		const IfcSchema::IfcRepresentation* representation = *et;

		std::string geotype = representation->data().getArgument(2)->toString();
		if (representation->data().getArgument(1)->toString() != "'Body'") { continue; }

		IfcSchema::IfcRepresentationItem* representationItems = *representation->Items().get()->begin();
		IfcGeom::IfcRepresentationShapeItems ob(kernel_->convert(representationItems));

		// move to OpenCASCADE
		for (size_t i = 0; i < ob.size(); i++)
		{
			TopoDS_Shape rShape = ob.at(i).Shape();
			gp_Trsf placement = ob.at(i).Placement().Trsf();
			rShape.Move(trsf * placement); // location in global space
			
			builder.Add(comp, rShape);
		}
	}
	return comp;
}

void helper::whipeObject(IfcSchema::IfcProduct* product)
{
	file_->removeEntity(product);

	// TODO check dependencies!
}

void helper::wipeObject(helper* data, int id)
{
	//TODO find the nested indx located in the file
	//TODO find if the nested idnx have other dependencies 
	//TODO remove objects at indx that do not have dependencies
}

void helper::writeToFile(std::string path)
{
	std::ofstream storageFile;
	storageFile.open(path);
	std::cout << "[INFO] Exporting file " << fileName_ << std::endl;
	storageFile << *file_;
	std::cout << "[INFO] Exported succesfully" << std::endl;
	storageFile.close();
}

void helperCluster::internaliseData()
{

	bool debug = true;

	if (!helperList[0]->getDepending())
	{
		helperList[0]->internalizeGeo();

		lllPoint_ = helperList[0]->getLllPoint();
		urrPoint_ = helperList[0]->getUrrPoint();
		originRot_ = helperList[0]->getRotation();

		if (debug)
		{
			std::cout << "cluster:" << std::endl;
			printPoint(lllPoint_);
			printPoint(urrPoint_);
			std::cout << originRot_ << std::endl;
		}

		return;
	}

	for (size_t i = 0; i < size_; i++)
	{
		auto helper = helperList[i];

		if (helper->getIsConstruct())
		{
			helper->internalizeGeo();
			lllPoint_ = helper->getLllPoint();
			urrPoint_ = helper->getUrrPoint();
			originRot_ = helper->getRotation();
		}
	}

	for (size_t i = 0; i < size_; i++)
	{
		auto helper = helperList[i];

		if (!helper->getHasGeo())
		{
			helper->internalizeGeo(originRot_);

			// update bbox if needed
			gp_Pnt addLllPoint = helper->getLllPoint();
			gp_Pnt addUrrPoint = helper->getUrrPoint();

			if (addLllPoint.X() < lllPoint_.X()) { lllPoint_.SetX(addLllPoint.X()); }
			if (addLllPoint.Y() < lllPoint_.Y()) { lllPoint_.SetY(addLllPoint.Y()); }
			if (addLllPoint.Z() < lllPoint_.Z()) { lllPoint_.SetZ(addLllPoint.Z()); }

			if (addUrrPoint.X() > urrPoint_.X()) { urrPoint_.SetX(addUrrPoint.X()); }
			if (addUrrPoint.Y() > urrPoint_.Y()) { urrPoint_.SetY(addUrrPoint.Y()); }
			if (addUrrPoint.Z() > urrPoint_.Z()) { urrPoint_.SetZ(addUrrPoint.Z()); }
		}
	}

	hasBbox_ = true;

	if (debug)
	{
		std::cout << "cluster:" << std::endl;
		printPoint(lllPoint_);
		printPoint(urrPoint_);
		std::cout << originRot_ << std::endl;
	}
}

void helperCluster::appendHelper(helper* data)
{
	helperList.emplace_back(data);
	size_++;
}