/********************************************************************************
 *                                                                              *
 * This file is part of IfcOpenShell.                                           *
 *                                                                              *
 * IfcOpenShell is free software: you can redistribute it and/or modify         *
 * it under the terms of the Lesser GNU General Public License as published by  *
 * the Free Software Foundation, either version 3.0 of the License, or          *
 * (at your option) any later version.                                          *
 *                                                                              *
 * IfcOpenShell is distributed in the hope that it will be useful,              *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of               *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 *
 * Lesser GNU General Public License for more details.                          *
 *                                                                              *
 * You should have received a copy of the Lesser GNU General Public License     *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.         *
 *                                                                              *
 ********************************************************************************/

 // TODO: Multiple schemas
#define IfcSchema Ifc4

#include <iostream>
#include <ifcparse/IfcFile.h>
#include <ifcparse/Ifc4.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#if USE_VLD
#include <vld.h>
#endif

int main(int argc, char** argv) {

	std::string sourcePath = "D:/Documents/Uni/Thesis/sources/Models/AC20-FZK-Haus.ifc";
	std::string exportPath = "D:/Documents/Uni/Thesis/sources/Models/exports/AC20-FZK-Haus.ifc";

	Logger::SetOutput(&std::cout, &std::cout);

	// get file path
	IfcParse::IfcFile workingFile(sourcePath);

	if (!workingFile.good()) {
		std::cout << "Unable to parse .ifc file" << std::endl;
		return 1;
	}
	else {
		std::cout << "Valid IFC file found" << std::endl;
	}

	// TODO seperate flat and angled slaps
	// TODO make buffer for flat slaps
	// TODO find buffer style for angled slaps
 	// TODO select objects within the buffer
	// TODO store to file

	IfcSchema::IfcBuildingElement::list::ptr elements = workingFile.instances_by_type<IfcSchema::IfcBuildingElement>();
	std::cout << "Found " << elements->size() << " elements " << std::endl;

	for (IfcSchema::IfcBuildingElement::list::it it = elements->begin(); it != elements->end(); ++it) {
		const IfcSchema::IfcBuildingElement* element = *it;

		const IfcSchema::IfcSlab* slab;

		if ((slab = element->as<IfcSchema::IfcSlab>()) != 0) {
			std::cout << element->data().toString() << std::endl;
		}

	}

	// write to file
	std::ofstream storageFile;
	storageFile.open(exportPath);
	std::cout << "exporting" << std::endl;
	storageFile << workingFile;
	std::cout << "exported succesfully" << std::endl;
	storageFile.close();

	/*
	for (IfcSchema::IfcBuildingElement::list::it it = elements->begin(); it != elements->end(); ++it) {

		const IfcSchema::IfcBuildingElement* element = *it;
		std::cout << element->data().toString() << std::endl;

		const IfcSchema::IfcWindow* window;
		if ((window = element->as<IfcSchema::IfcWindow>()) != 0) {
			if (window->hasOverallWidth() && window->hasOverallHeight()) {
				const double area = window->OverallWidth() * window->OverallHeight();
				std::cout << "The area of this window is " << area << std::endl;
			}
		}

	}*/


	std::cout << "last line executed" << std::endl;
	return 0;

}