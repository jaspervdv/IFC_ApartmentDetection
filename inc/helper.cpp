#include "helper.h"

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
